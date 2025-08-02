// smart_collector.h
#pragma once

#include "../media/slicer.h"
#include "../media/erasure_coder.h"
#include <map>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <functional>

using FrameData = std::vector<uint8_t>;
using OnFrameReadyCallback = std::function<void(FrameData)>;

class SmartCollector {
private:
    struct FrameBuffer {
        std::chrono::steady_clock::time_point first_chunk_arrival;
        std::vector<ChunkPtr> received_chunks;
        uint16_t k = 0;
        uint16_t r = 0;
        int data_chunks_count = 0;
        bool reconstructed = false;
    };

public:
    SmartCollector(OnFrameReadyCallback cb, std::shared_ptr<ErasureCoder> fec_decoder);
    ~SmartCollector();
    
    void start();
    void stop();
    void pushChunk(ChunkPtr chunk);

private:
    void flush_loop();

    std::map<uint32_t, FrameBuffer> frame_map_;
    std::mutex map_mutex_;
    std::thread flush_thread_;
    std::atomic<bool> running_{false};
    
    OnFrameReadyCallback on_frame_ready_;
    std::shared_ptr<ErasureCoder> fec_decoder_;
    
    const std::chrono::milliseconds JITTER_WINDOW{50};
    const std::chrono::milliseconds FLUSH_INTERVAL{25};
};