#include "smart_collector.hpp"
#include <algorithm>
#include <iostream>
#include <chrono>
#include <cstring>

// NovaEngine Ultra Stream - Time-Windowed Jitter Buffer
// <50ms latency with intelligent frame reconstruction

// Global collector instance
SmartCollector* g_collector = nullptr;

SmartCollector::SmartCollector(int window_ms, int k_chunks, int r_chunks)
    : running_(false), window_timeout_(window_ms), k_chunks_(k_chunks), r_chunks_(r_chunks) {
    
    // Initialize statistics
    stats_ = {};
    
    std::cout << "[Collector] Initialized with window=" << window_ms << "ms, k=" << k_chunks 
              << ", r=" << r_chunks << std::endl;
}

SmartCollector::~SmartCollector() {
    stopFlushThread();
}

void SmartCollector::addChunk(const ChunkPacket& chunk) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    uint32_t frame_id = chunk.frame_id;
    
    // Check if this is a duplicate chunk
    auto it = frame_buffers_.find(frame_id);
    if (it != frame_buffers_.end() && it->second.chunk_received[chunk.chunk_id]) {
        stats_.chunks_duplicate++;
        return;
    }
    
    // Create new frame buffer if needed
    if (it == frame_buffers_.end()) {
        frame_buffers_.emplace(frame_id, FrameBuffer(frame_id, chunk.total_chunks));
        stats_.frames_received++;
    }
    
    FrameBuffer& buffer = frame_buffers_[frame_id];
    
    // Add chunk to buffer
    if (chunk.chunk_id < buffer.total_chunks) {
        std::vector<uint8_t> chunk_data(chunk.data, chunk.data + chunk.data_size);
        buffer.chunks[chunk.chunk_id] = chunk_data;
        buffer.chunk_received[chunk.chunk_id] = true;
        buffer.received_chunks++;
        
        stats_.chunks_received++;
        
        std::cout << "[Collector] Added chunk " << (int)chunk.chunk_id << "/" << buffer.total_chunks 
                  << " for frame " << frame_id << std::endl;
    }
}

std::vector<uint8_t> SmartCollector::getCompleteFrame(uint32_t frame_id) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    auto it = frame_buffers_.find(frame_id);
    if (it == frame_buffers_.end()) {
        return {};
    }
    
    FrameBuffer& buffer = it->second;
    
    if (canReconstructFrame(buffer)) {
        std::vector<uint8_t> frame_data;
        reconstructFrame(frame_id, frame_data);
        
        // Remove frame from buffer
        frame_buffers_.erase(it);
        
        // Update statistics
        updateStats(buffer, true);
        
        return frame_data;
    }
    
    return {};
}

bool SmartCollector::isFrameReady(uint32_t frame_id) const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    auto it = frame_buffers_.find(frame_id);
    if (it == frame_buffers_.end()) {
        return false;
    }
    
    return canReconstructFrame(it->second);
}

void SmartCollector::startFlushThread(std::function<void(const std::vector<uint8_t>&, uint32_t)> callback) {
    if (running_) return;
    
    frame_callback_ = callback;
    running_ = true;
    flush_thread_ = std::thread(&SmartCollector::flushThread, this);
    
    std::cout << "[Collector] Flush thread started with " << window_timeout_.count() << "ms window" << std::endl;
}

void SmartCollector::stopFlushThread() {
    if (!running_) return;
    
    running_ = false;
    if (flush_thread_.joinable()) {
        flush_thread_.join();
    }
    
    std::cout << "[Collector] Flush thread stopped" << std::endl;
}

void SmartCollector::setWindowTimeout(int ms) {
    window_timeout_ = std::chrono::milliseconds(ms);
}

SmartCollector::CollectorStats SmartCollector::getStats() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    CollectorStats stats = stats_;
    stats.buffer_size = frame_buffers_.size();
    
    // Calculate completion rate
    if (stats.frames_received > 0) {
        stats.completion_rate = static_cast<double>(stats.frames_completed) / stats.frames_received;
    }
    
    return stats;
}

void SmartCollector::cleanupExpired() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    auto now = std::chrono::steady_clock::now();
    std::vector<uint32_t> expired_frames;
    
    for (const auto& pair : frame_buffers_) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - pair.second.timestamp).count();
        
        if (elapsed > window_timeout_.count()) {
            expired_frames.push_back(pair.first);
        }
    }
    
    for (uint32_t frame_id : expired_frames) {
        auto it = frame_buffers_.find(frame_id);
        if (it != frame_buffers_.end()) {
            updateStats(it->second, false);
            frame_buffers_.erase(it);
            stats_.frames_expired++;
            
            std::cout << "[Collector] Frame " << frame_id << " expired after " 
                      << window_timeout_.count() << "ms" << std::endl;
        }
    }
}

size_t SmartCollector::getBufferSize() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return frame_buffers_.size();
}

std::vector<uint32_t> SmartCollector::getPendingFrames() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    std::vector<uint32_t> pending;
    for (const auto& pair : frame_buffers_) {
        pending.push_back(pair.first);
    }
    
    return pending;
}

void SmartCollector::flushThread() {
    while (running_) {
        // Check for complete frames
        std::vector<std::pair<uint32_t, std::vector<uint8_t>>> completed_frames;
        
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            
            for (auto it = frame_buffers_.begin(); it != frame_buffers_.end();) {
                if (canReconstructFrame(it->second)) {
                    std::vector<uint8_t> frame_data;
                    reconstructFrame(it->first, frame_data);
                    completed_frames.emplace_back(it->first, std::move(frame_data));
                    
                    updateStats(it->second, true);
                    it = frame_buffers_.erase(it);
                } else {
                    ++it;
                }
            }
        }
        
        // Call callback for completed frames
        for (const auto& pair : completed_frames) {
            if (frame_callback_) {
                frame_callback_(pair.second, pair.first);
            }
        }
        
        // Clean up expired frames
        cleanupExpired();
        
        // Sleep for a short interval
        std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 5ms intervals
    }
}

bool SmartCollector::canReconstructFrame(const FrameBuffer& buffer) const {
    // Need at least k chunks to reconstruct
    return buffer.received_chunks >= k_chunks_;
}

std::vector<uint8_t> SmartCollector::reconstructFrame(uint32_t frame_id, std::vector<uint8_t>& frame_data) {
    auto it = frame_buffers_.find(frame_id);
    if (it == frame_buffers_.end()) {
        return {};
    }
    
    FrameBuffer& buffer = it->second;
    
    // Check if we have enough chunks for reconstruction
    if (buffer.chunks.size() < k_chunks_) {
        return {};
    }
    
    // Separate data and parity chunks
    std::vector<std::vector<uint8_t>> data_chunks;
    std::vector<std::vector<uint8_t>> parity_chunks;
    
    for (size_t i = 0; i < buffer.chunks.size(); i++) {
        if (i < k_chunks_) {
            data_chunks.push_back(buffer.chunks[i]);
        } else {
            parity_chunks.push_back(buffer.chunks[i]);
        }
    }
    
    // If we have all data chunks, no need for FEC decode
    if (data_chunks.size() == k_chunks_) {
        // Concatenate data chunks
        frame_data.clear();
        for (const auto& chunk : data_chunks) {
            frame_data.insert(frame_data.end(), chunk.begin(), chunk.end());
        }
        
        // Remove padding (last chunk might be padded)
        if (!frame_data.empty()) {
            // Find the last non-zero byte to determine actual size
            size_t actual_size = frame_data.size();
            while (actual_size > 0 && frame_data[actual_size - 1] == 0) {
                actual_size--;
            }
            frame_data.resize(actual_size);
        }
        
        std::cout << "[Collector] Reconstructed frame " << frame_id 
                  << " from " << data_chunks.size() << "/" << k_chunks_ << " data chunks (" 
                  << frame_data.size() << " bytes)" << std::endl;
        return frame_data;
    }
    
    // For now, just concatenate available data chunks
    frame_data.clear();
    for (const auto& chunk : data_chunks) {
        frame_data.insert(frame_data.end(), chunk.begin(), chunk.end());
    }
    
    std::cout << "[Collector] Partial reconstruction frame " << frame_id 
              << " from " << data_chunks.size() << "/" << k_chunks_ << " data chunks (" 
              << frame_data.size() << " bytes)" << std::endl;
    
    return frame_data;
}

void SmartCollector::updateStats(const FrameBuffer& buffer, bool completed) {
    if (completed) {
        stats_.frames_completed++;
        
        // Calculate latency only for completed frames
        double latency = calculateLatency(buffer);
        if (latency >= 0) {  // Only update if latency is valid
            if (stats_.frames_completed == 1) {
                stats_.avg_latency_ms = latency;
            } else {
                stats_.avg_latency_ms = (stats_.avg_latency_ms * (stats_.frames_completed - 1) + latency) / stats_.frames_completed;
            }
        }
    }
}

double SmartCollector::calculateLatency(const FrameBuffer& buffer) const {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - buffer.timestamp);
    return elapsed.count();
}

// Global interface functions
void initCollector(int window_ms, int k, int r) {
    if (g_collector) {
        delete g_collector;
    }
    
    g_collector = new SmartCollector(window_ms, k, r);
    std::cout << "[Collector] ✅ Global collector initialized" << std::endl;
}

void addChunkToCollector(const ChunkPacket& chunk) {
    if (!g_collector) {
        throw std::runtime_error("Collector not initialized");
    }
    
    g_collector->addChunk(chunk);
}

void startCollectorFlush(std::function<void(const std::vector<uint8_t>&, uint32_t)> callback) {
    if (!g_collector) {
        throw std::runtime_error("Collector not initialized");
    }
    
    g_collector->startFlushThread(callback);
}

void shutdownCollector() {
    if (g_collector) {
        g_collector->stopFlushThread();
        delete g_collector;
        g_collector = nullptr;
    }
    
    std::cout << "[Collector] Collector shutdown complete" << std::endl;
}

