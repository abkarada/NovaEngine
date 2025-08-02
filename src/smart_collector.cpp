#include "smart_collector.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include <algorithm>
#include <deque>

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;
constexpr int JITTER_TIMEOUT_MS = 50;  // Increased from 15ms to 50ms for better tolerance
constexpr int MAX_FRAME_AGE_MS = 200;  // Maximum age before dropping frame
constexpr int FLUSH_INTERVAL_MS = 10;  // More frequent flushing

SmartFrameCollector::SmartFrameCollector(FrameReadyCallback cb, int k, int r)
    : callback(std::move(cb)), fec(k, r), k_(k), r_(r) {
    timeout_ms_ = JITTER_TIMEOUT_MS;
    flush_thread = std::thread([this]() {
        while (!stop_flag) {
            std::this_thread::sleep_for(std::chrono::milliseconds(FLUSH_INTERVAL_MS));
            flush_expired_frames();
        }
    });
}

SmartFrameCollector::~SmartFrameCollector() {
    stop_flag = true;
    if (flush_thread.joinable()) flush_thread.join();
}

void SmartFrameCollector::handle(ChunkPacket pkt) {
    if (pkt.total_chunks == 0 || pkt.chunk_id >= pkt.total_chunks) {
        std::cerr << "[COLLECTOR] Invalid packet: frame=" << pkt.frame_id 
                 << ", chunk=" << (int)pkt.chunk_id << "/" << (int)pkt.total_chunks << std::endl;
        return;
    }

    auto& frame = frame_buffer[pkt.frame_id];

    // Initialize frame structure if first time
    if (frame.chunks.empty()) {
        frame.chunks.resize(pkt.total_chunks);
        frame.received_flags.resize(pkt.total_chunks, false);
        frame.total_chunks = pkt.total_chunks;
        frame.arrival_time = Clock::now();
        std::cout << "[COLLECTOR] New frame " << pkt.frame_id 
                 << " with " << pkt.total_chunks << " chunks" << std::endl;
    }

    // Duplicate check
    if (frame.received_flags[pkt.chunk_id]) {
        std::cout << "[COLLECTOR] Duplicate chunk " << (int)pkt.chunk_id 
                 << " for frame " << pkt.frame_id << std::endl;
        return;
    }

    frame.chunks[pkt.chunk_id] = std::move(pkt.payload);
    frame.received_flags[pkt.chunk_id] = true;
    frame.received_chunks++;
    frame.last_update = Clock::now();

    std::cout << "[COLLECTOR] Frame " << pkt.frame_id << ": " 
             << frame.received_chunks << "/" << pkt.total_chunks 
             << " chunks received" << std::endl;

    // Try to decode immediately if we have enough chunks
    if (frame.received_chunks >= static_cast<size_t>(k_)) {
        std::cout << "[COLLECTOR] Attempting FEC decode for frame " << pkt.frame_id 
                 << " (received: " << frame.received_chunks << "/" << pkt.total_chunks 
                 << ", k=" << k_ << ", r=" << r_ << ")" << std::endl;
        
        std::vector<uint8_t> recovered;
        if (fec.decode(frame.chunks, frame.received_flags, recovered)) {
            std::cout << "[COLLECTOR] ✅ FEC decode SUCCESS for frame " << pkt.frame_id 
                     << ", recovered " << recovered.size() << " bytes" << std::endl;
            
            // Check frame age before delivering
            auto now = Clock::now();
            auto frame_age = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - frame.arrival_time).count();
            
            if (frame_age < MAX_FRAME_AGE_MS) {
                std::cout << "[COLLECTOR] 🎯 Delivering frame " << pkt.frame_id 
                         << " to decoder (age: " << frame_age << "ms)" << std::endl;
                callback(recovered);
            } else {
                std::cerr << "[COLLECTOR] ❌ Dropping old frame " << pkt.frame_id 
                         << " (age: " << frame_age << "ms)" << std::endl;
            }
            frame_buffer.erase(pkt.frame_id);
        } else {
            std::cerr << "[COLLECTOR] ❌ FEC decode FAILED for frame " << pkt.frame_id 
                     << " (received: " << frame.received_chunks << "/" << pkt.total_chunks << ")" << std::endl;
            
            // Log which chunks are missing
            std::string missing_chunks;
            for (size_t i = 0; i < frame.received_flags.size(); ++i) {
                if (!frame.received_flags[i]) {
                    if (!missing_chunks.empty()) missing_chunks += ", ";
                    missing_chunks += std::to_string(i);
                }
            }
            std::cout << "[COLLECTOR] Missing chunks: [" << missing_chunks << "]" << std::endl;
        }
    }
}

void SmartFrameCollector::flush_expired_frames() {
    TimePoint now = Clock::now();
    std::vector<uint16_t> to_finalize;
    std::vector<uint16_t> to_drop;

    for (auto& [fid, frame] : frame_buffer) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - frame.last_update).count();
        auto frame_age = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - frame.arrival_time).count();
        
        // Drop frames that are too old
        if (frame_age > MAX_FRAME_AGE_MS) {
            to_drop.push_back(fid);
            continue;
        }
        
        // Try to finalize frames that have enough chunks and have timed out
        if (frame.received_chunks >= static_cast<size_t>(k_) && elapsed > timeout_ms_) {
            to_finalize.push_back(fid);
        }
    }

    // Drop old frames
    for (uint16_t fid : to_drop) {
        std::cerr << "[COLLECTOR] Dropping expired frame " << fid << std::endl;
        frame_buffer.erase(fid);
    }

    // Try to decode frames that have timed out
    for (uint16_t fid : to_finalize) {
        auto& frame = frame_buffer[fid];
        std::vector<uint8_t> recovered;
        
        if (fec.decode(frame.chunks, frame.received_flags, recovered)) {
            auto frame_age = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - frame.arrival_time).count();
            
            if (frame_age < MAX_FRAME_AGE_MS) {
                callback(recovered);
            } else {
                std::cerr << "[COLLECTOR] Dropping old frame " << fid 
                         << " (age: " << frame_age << "ms)" << std::endl;
            }
        } else {
            std::cerr << "[COLLECTOR] FEC decode failed for frame " << fid 
                     << " (received: " << frame.received_chunks << "/" << frame.total_chunks << ")" << std::endl;
        }
        
        frame_buffer.erase(fid);
    }
    
    // Clean up very old frames to prevent memory leaks
    if (frame_buffer.size() > 100) {
        std::vector<std::pair<uint16_t, TimePoint>> frame_ages;
        for (const auto& [fid, frame] : frame_buffer) {
            frame_ages.emplace_back(fid, frame.arrival_time);
        }
        
        std::sort(frame_ages.begin(), frame_ages.end(), 
                 [](const auto& a, const auto& b) { return a.second < b.second; });
        
        // Keep only the 50 most recent frames
        for (size_t i = 50; i < frame_ages.size(); ++i) {
            frame_buffer.erase(frame_ages[i].first);
        }
    }
}

