#include "slicer.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <cstring>

// NovaEngine Ultra Stream - MTU-Aware Frame Slicer
// Intelligent frame chunking with Reed-Solomon FEC preparation

// Global slicer instance
FrameSlicer* g_slicer = nullptr;

FrameSlicer::FrameSlicer(int chunk_size, int k_chunks, int r_chunks)
    : chunk_size_(chunk_size), k_chunks_(k_chunks), r_chunks_(r_chunks) {
    std::cout << "[Slicer] Initialized with chunk_size=" << chunk_size_ 
              << ", k=" << k_chunks_ << ", r=" << r_chunks_ << std::endl;
}

FrameSlicer::~FrameSlicer() {
}

std::vector<ChunkPacket> FrameSlicer::sliceFrame(const std::vector<uint8_t>& frame_data, 
                                                uint32_t frame_id, uint64_t timestamp) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    std::vector<ChunkPacket> chunks;
    int total_chunks = k_chunks_ + r_chunks_;
    
    // Pad data to fit exactly k chunks
    std::vector<uint8_t> padded_data = padChunk(frame_data, k_chunks_ * chunk_size_);
    
    // Create k data chunks
    for (int i = 0; i < k_chunks_; ++i) {
        size_t offset = i * chunk_size_;
        std::vector<uint8_t> chunk_data(padded_data.begin() + offset, 
                                       padded_data.begin() + offset + chunk_size_);
        
        ChunkPacket packet = createChunkPacket(frame_id, i, total_chunks, chunk_data, timestamp);
        chunks.push_back(packet);
    }
    
    // Create r parity chunks (will be filled by erasure coder)
    for (int i = k_chunks_; i < total_chunks; ++i) {
        std::vector<uint8_t> empty_chunk(chunk_size_, 0);
        ChunkPacket packet = createChunkPacket(frame_id, i, total_chunks, empty_chunk, timestamp);
        chunks.push_back(packet);
    }
    
    // Update statistics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    last_stats_.frame_id = frame_id;
    last_stats_.total_chunks = total_chunks;
    last_stats_.data_chunks = k_chunks_;
    last_stats_.parity_chunks = r_chunks_;
    last_stats_.original_size = frame_data.size();
    last_stats_.chunked_size = chunks.size() * sizeof(ChunkPacket);
    last_stats_.slice_time_us = duration.count();
    
    std::cout << "[Slicer] Frame " << frame_id << " sliced into " << total_chunks 
              << " chunks (" << k_chunks_ << " data + " << r_chunks_ << " parity) in " 
              << duration.count() << "μs" << std::endl;
    
    return chunks;
}

std::vector<uint8_t> FrameSlicer::reconstructFrame(const std::vector<ChunkPacket>& chunks) {
    if (chunks.empty()) return {};
    
    uint32_t frame_id = chunks[0].frame_id;
    std::vector<uint8_t> reconstructed_data;
    
    // Sort chunks by chunk_id
    std::vector<ChunkPacket> sorted_chunks = chunks;
    std::sort(sorted_chunks.begin(), sorted_chunks.end(), 
              [](const ChunkPacket& a, const ChunkPacket& b) {
                  return a.chunk_id < b.chunk_id;
              });
    
    // Reconstruct from data chunks only
    for (const auto& chunk : sorted_chunks) {
        if (chunk.chunk_id < k_chunks_) {
            reconstructed_data.insert(reconstructed_data.end(), 
                                     chunk.data, 
                                     chunk.data + chunk.data_size);
        }
    }
    
    std::cout << "[Slicer] Frame " << frame_id << " reconstructed: " 
              << reconstructed_data.size() << " bytes" << std::endl;
    
    return reconstructed_data;
}

FrameSlicer::SliceStats FrameSlicer::getLastSliceStats() const {
    return last_stats_;
}

void FrameSlicer::setChunkSize(int size) {
    chunk_size_ = size;
    std::cout << "[Slicer] Chunk size updated to " << chunk_size_ << " bytes" << std::endl;
}

void FrameSlicer::setFECParams(int k, int r) {
    k_chunks_ = k;
    r_chunks_ = r;
    std::cout << "[Slicer] FEC parameters updated: k=" << k_chunks_ << ", r=" << r_chunks_ << std::endl;
}

ChunkPacket FrameSlicer::createChunkPacket(uint32_t frame_id, uint16_t chunk_id, 
                                          uint16_t total_chunks, const std::vector<uint8_t>& data,
                                          uint64_t timestamp) {
    ChunkPacket packet{};
    packet.frame_id = frame_id;
    packet.chunk_id = chunk_id;
    packet.total_chunks = total_chunks;
    packet.timestamp = timestamp;
    packet.path_id = 0;
    packet.priority = (chunk_id < k_chunks_) ? 1 : 0;  // Data chunks have higher priority
    
    // Copy data to packet
    packet.data_size = std::min(static_cast<uint16_t>(data.size()), 
                               static_cast<uint16_t>(sizeof(packet.data)));
    std::memcpy(packet.data, data.data(), packet.data_size);
    
    return packet;
}

std::vector<uint8_t> FrameSlicer::padChunk(const std::vector<uint8_t>& data, int target_size) {
    std::vector<uint8_t> padded = data;
    
    if (padded.size() < static_cast<size_t>(target_size)) {
        // Pad with zeros to reach target size
        padded.resize(target_size, 0);
    } else if (padded.size() > static_cast<size_t>(target_size)) {
        // Truncate if too large
        padded.resize(target_size);
    }
    
    return padded;
}

// Global interface functions
void initSlicer(int chunk_size, int k, int r) {
    if (g_slicer) {
        delete g_slicer;
    }
    
    g_slicer = new FrameSlicer(chunk_size, k, r);
    std::cout << "[Slicer] ✅ Global slicer initialized" << std::endl;
}

std::vector<ChunkPacket> sliceFrame(const std::vector<uint8_t>& frame_data, 
                                   uint32_t frame_id, uint64_t timestamp) {
    if (!g_slicer) {
        throw std::runtime_error("Slicer not initialized");
    }
    
    return g_slicer->sliceFrame(frame_data, frame_id, timestamp);
}

void shutdownSlicer() {
    if (g_slicer) {
        delete g_slicer;
        g_slicer = nullptr;
    }
    
    std::cout << "[Slicer] Slicer shutdown complete" << std::endl;
}

