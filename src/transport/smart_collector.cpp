#include "smart_collector.h"
#include <iostream>

SmartCollector::SmartCollector(OnFrameReadyCallback cb, std::shared_ptr<ErasureCoder> fec_decoder)
    : on_frame_ready_(cb), fec_decoder_(fec_decoder) {
    std::cout << "SmartCollector initialized" << std::endl;
}

SmartCollector::~SmartCollector() {
    stop();
    std::cout << "SmartCollector destroyed" << std::endl;
}

void SmartCollector::start() {
    running_ = true;
    flush_thread_ = std::thread(&SmartCollector::flush_loop, this);
    std::cout << "SmartCollector started" << std::endl;
}

void SmartCollector::stop() {
    running_ = false;
    if (flush_thread_.joinable()) {
        flush_thread_.join();
    }
    std::cout << "SmartCollector stopped" << std::endl;
}

void SmartCollector::pushChunk(ChunkPtr chunk) {
    if (!chunk) return;
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    uint32_t frame_id = chunk->header.frame_id;
    
    if (frame_map_.find(frame_id) == frame_map_.end()) {
        frame_map_[frame_id] = FrameBuffer{};
        frame_map_[frame_id].first_chunk_arrival = std::chrono::steady_clock::now();
        frame_map_[frame_id].k = chunk->header.k;
        frame_map_[frame_id].r = chunk->header.r;
    }
    
    auto& buffer = frame_map_[frame_id];
    if (chunk->header.chunk_id < buffer.k) {
        buffer.data_chunks_count++;
    }
    buffer.received_chunks.push_back(chunk);
    
    std::cout << "Received chunk " << chunk->header.chunk_id 
              << " for frame " << frame_id << std::endl;
}

void SmartCollector::flush_loop() {
    while (running_) {
        std::this_thread::sleep_for(FLUSH_INTERVAL);
        
        std::lock_guard<std::mutex> lock(map_mutex_);
        auto now = std::chrono::steady_clock::now();
        
        for (auto it = frame_map_.begin(); it != frame_map_.end();) {
            auto& buffer = it->second;
            auto age = now - buffer.first_chunk_arrival;
            
            if (age > JITTER_WINDOW || buffer.data_chunks_count >= buffer.k) {
                // Try to reconstruct frame
                if (buffer.data_chunks_count >= buffer.k && !buffer.reconstructed) {
                    // Basic reconstruction - just concatenate data chunks
                    std::vector<uint8_t> frame_data;
                    for (auto& chunk : buffer.received_chunks) {
                        if (chunk->header.chunk_id < buffer.k) {
                            frame_data.insert(frame_data.end(), 
                                            chunk->payload.begin(), 
                                            chunk->payload.end());
                        }
                    }
                    
                    if (on_frame_ready_) {
                        on_frame_ready_(frame_data);
                    }
                    buffer.reconstructed = true;
                }
                
                it = frame_map_.erase(it);
            } else {
                ++it;
            }
        }
    }
} 