// src/core/engine.h
#pragma once

#include "../media/ffmpeg_encoder.h"
#include "../media/slicer.h"
#include "../media/erasure_coder.h"
#include "../network/sender_receiver.h"
#include "../network/scheduler.h"
#include "../network/path_monitor.h"
#include "../transport/smart_collector.h"
#include "../common/logger.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>
#include <atomic>

// Forward declarations
using FrameData = std::vector<uint8_t>;

class Engine {
public:
    Engine(const std::string& remote_ip, uint16_t base_remote_port, size_t num_tunnels);
    ~Engine();

    void run();
    void stop();

private:
    void initialize_senders(const std::string& remote_ip, uint16_t base_remote_port);
    void send_frame(const std::vector<uint8_t>& encoded_frame);
    void on_chunk_received(ChunkPtr chunk, size_t path_index);
    void on_frame_ready(FrameData frame_data);
    
    std::atomic<bool> running_{false};
    uint32_t frame_id_counter_ = 0;

    // --- Configuration ---
    const size_t num_tunnels_;
    const int k_ = 10; // 10 data chunks
    const int r_ = 4;  // 4 parity chunks

    // --- Modules ---
    std::unique_ptr<FFmpegEncoder> encoder_;
    std::shared_ptr<ErasureCoder> erasure_coder_;
    std::shared_ptr<Scheduler> scheduler_;
    std::unique_ptr<SmartCollector> collector_;
    std::unique_ptr<PathMonitor> path_monitor_;
    std::vector<std::unique_ptr<SenderReceiver>> sender_receivers_;
};