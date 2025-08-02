#pragma once

#include "udp_sender.hpp"
#include <map>
#include <vector>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>
#include <functional>

// NovaEngine Ultra Stream - Time-Windowed Jitter Buffer
// <50ms latency with intelligent frame reconstruction

class SmartCollector {
public:
    SmartCollector(int window_ms = 50, int k_chunks = 6, int r_chunks = 2);
    ~SmartCollector();
    
    // Add incoming chunk to buffer
    void addChunk(const ChunkPacket& chunk);
    
    // Get complete frame if available
    std::vector<uint8_t> getCompleteFrame(uint32_t frame_id);
    
    // Check if frame is ready for processing
    bool isFrameReady(uint32_t frame_id) const;
    
    // Start background flush thread
    void startFlushThread(std::function<void(const std::vector<uint8_t>&, uint32_t)> callback);
    
    // Stop flush thread
    void stopFlushThread();
    
    // Set window timeout
    void setWindowTimeout(int ms);
    
    // Get collector statistics
    struct CollectorStats {
        uint32_t frames_received;
        uint32_t frames_completed;
        uint32_t frames_expired;
        uint32_t chunks_received;
        uint32_t chunks_duplicate;
        uint32_t chunks_lost;
        double avg_latency_ms;
        double completion_rate;
        uint64_t buffer_size;
    };
    
    CollectorStats getStats() const;
    
    // Clear expired frames
    void cleanupExpired();
    
    // Get buffer status
    size_t getBufferSize() const;
    std::vector<uint32_t> getPendingFrames() const;

private:
    struct FrameBuffer {
        std::vector<std::vector<uint8_t>> chunks;
        std::vector<bool> chunk_received;
        std::chrono::steady_clock::time_point timestamp;
        uint32_t frame_id;
        int total_chunks;
        int received_chunks;
        
        FrameBuffer() : frame_id(0), total_chunks(0), received_chunks(0) {
            timestamp = std::chrono::steady_clock::now();
        }
        
        FrameBuffer(uint32_t id, int total) 
            : frame_id(id), total_chunks(total), received_chunks(0) {
            chunks.resize(total);
            chunk_received.resize(total, false);
            timestamp = std::chrono::steady_clock::now();
        }
    };
    
    std::map<uint32_t, FrameBuffer> frame_buffers_;
    mutable std::mutex buffer_mutex_;
    std::atomic<bool> running_;
    std::thread flush_thread_;
    std::chrono::milliseconds window_timeout_;
    int k_chunks_;
    int r_chunks_;
    CollectorStats stats_;
    std::function<void(const std::vector<uint8_t>&, uint32_t)> frame_callback_;
    
    // Background flush thread function
    void flushThread();
    
    // Check if frame can be reconstructed
    bool canReconstructFrame(const FrameBuffer& buffer) const;
    
    // Reconstruct frame from chunks
    std::vector<uint8_t> reconstructFrame(const FrameBuffer& buffer);
    
    // Update statistics
    void updateStats(const FrameBuffer& buffer, bool completed);
    
    // Calculate frame latency
    double calculateLatency(const FrameBuffer& buffer) const;
};

// Global collector interface
extern SmartCollector* g_collector;

void initCollector(int window_ms = 50, int k = 6, int r = 2);
void addChunkToCollector(const ChunkPacket& chunk);
void startCollectorFlush(std::function<void(const std::vector<uint8_t>&, uint32_t)> callback);
void shutdownCollector();

