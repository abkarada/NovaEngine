#include "engine.h"
#include <iostream>

Engine::Engine(const std::string& remote_ip, uint16_t base_remote_port, size_t num_tunnels)
    : num_tunnels_(num_tunnels) {
    // Initialize components
    encoder_ = std::make_unique<FFmpegEncoder>(1920, 1080, 30, 2000);
    erasure_coder_ = std::make_shared<ErasureCoder>();
    scheduler_ = std::make_shared<Scheduler>();
    path_monitor_ = std::make_unique<PathMonitor>();
    
    // Initialize collector with callback
    collector_ = std::make_unique<SmartCollector>(
        [this](FrameData frame_data) { this->on_frame_ready(frame_data); },
        erasure_coder_
    );
    
    initialize_senders(remote_ip, base_remote_port);
}

Engine::~Engine() {
    stop();
}

void Engine::run() {
    running_ = true;
    std::cout << "Engine started" << std::endl;
}

void Engine::stop() {
    running_ = false;
    std::cout << "Engine stopped" << std::endl;
}

void Engine::initialize_senders(const std::string& remote_ip, uint16_t base_remote_port) {
    for (size_t i = 0; i < num_tunnels_; ++i) {
        auto sender = std::make_unique<SenderReceiver>(
            base_remote_port + i,
            remote_ip,
            base_remote_port + i,
            [this, i](ChunkPtr chunk) { this->on_chunk_received(chunk, i); }
        );
        sender_receivers_.push_back(std::move(sender));
    }
}

void Engine::send_frame(const std::vector<uint8_t>& encoded_frame) {
    // Implementation for sending frame
    std::cout << "Sending frame of size: " << encoded_frame.size() << std::endl;
}

void Engine::on_chunk_received(ChunkPtr chunk, size_t path_index) {
    collector_->pushChunk(chunk);
}

void Engine::on_frame_ready(FrameData frame_data) {
    // Handle reconstructed frame
    std::cout << "Frame ready, size: " << frame_data.size() << std::endl;
} 