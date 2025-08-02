#include "sender_receiver.hpp"
#include "slicer.hpp"
#include "smart_collector.hpp"
#include <iostream>
#include <chrono>
#include <thread>

// NovaEngine Ultra Stream - Zero-Copy Sender/Receiver
// Ultra-low latency bidirectional I/O with std::move optimization

// Global sender/receiver instance
SenderReceiver* g_sender_receiver = nullptr;

SenderReceiver::SenderReceiver() : running_(false) {
    // Initialize statistics
    stats_ = {};
}

SenderReceiver::~SenderReceiver() {
    stop();
}

bool SenderReceiver::init(const std::vector<int>& local_ports, 
                         const std::string& target_ip, 
                         const std::vector<int>& target_ports) {
    // Initialize UDP sender with tunnels
    if (!sender_.initTunnels(local_ports)) {
        std::cerr << "[SenderReceiver] Failed to initialize UDP tunnels" << std::endl;
        return false;
    }
    
    // Set target addresses
    sender_.setTargets(target_ip, target_ports);
    
    std::cout << "[SenderReceiver] Initialized with " << local_ports.size() 
              << " local ports and " << target_ports.size() << " target ports" << std::endl;
    
    return true;
}

bool SenderReceiver::sendFrame(const std::vector<uint8_t>& frame_data, uint32_t frame_id) {
    if (!running_) {
        std::cerr << "[SenderReceiver] Not running, cannot send frame" << std::endl;
        return false;
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Create timestamp for this frame
    uint64_t timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    
    // Slice frame into chunks
    std::vector<ChunkPacket> chunks = sliceFrame(frame_data, frame_id, timestamp);
    
    // Send chunks through weighted scheduler
    int chunks_sent = 0;
    for (const auto& chunk : chunks) {
        ssize_t sent = sender_.sendChunk(chunk);
        if (sent > 0) {
            chunks_sent++;
            stats_.chunks_sent++;
            stats_.bytes_sent += sent;
        }
    }
    
    // Update statistics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto latency = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    updateSendStats(frame_id, latency.count());
    
    stats_.frames_sent++;
    
    std::cout << "[SenderReceiver] Sent frame " << frame_id << ": " 
              << chunks_sent << "/" << chunks.size() << " chunks, "
              << frame_data.size() << " bytes in " << latency.count() << "μs" << std::endl;
    
    return chunks_sent > 0;
}

void SenderReceiver::setReceiveCallback(std::function<void(const std::vector<uint8_t>&, uint32_t)> callback) {
    receive_callback_ = callback;
}

void SenderReceiver::start() {
    if (running_) return;
    
    running_ = true;
    
    // Start sender thread
    sender_thread_ = std::thread(&SenderReceiver::senderThread, this);
    
    // Start receiver thread
    receiver_thread_ = std::thread(&SenderReceiver::receiverThread, this);
    
    // Start bidirectional receive on UDP sender
    sender_.startBidirectionalReceive(
        [this](const ChunkPacket& chunk, int tunnel_id) {
            handleIncomingChunk(chunk, tunnel_id);
        }
    );
    
    std::cout << "[SenderReceiver] Started sender and receiver threads" << std::endl;
}

void SenderReceiver::stop() {
    if (!running_) return;
    
    running_ = false;
    
    // Stop UDP sender bidirectional receive
    sender_.stopBidirectionalReceive();
    
    // Join threads
    if (sender_thread_.joinable()) {
        sender_thread_.join();
    }
    
    if (receiver_thread_.joinable()) {
        receiver_thread_.join();
    }
    
    std::cout << "[SenderReceiver] Stopped all threads" << std::endl;
}

SenderReceiver::SRStats SenderReceiver::getStats() const {
    SRStats stats = stats_;
    
    // Calculate throughput
    if (stats.bytes_sent > 0) {
        stats.throughput_mbps = (stats.bytes_sent * 8.0) / (1000000.0);  // Convert to Mbps
    }
    
    return stats;
}

void SenderReceiver::senderThread() {
    std::cout << "[SenderReceiver] Sender thread started" << std::endl;
    
    while (running_) {
        // This thread can be used for periodic tasks or background processing
        // For now, it's mainly for coordination
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "[SenderReceiver] Sender thread stopped" << std::endl;
}

void SenderReceiver::receiverThread() {
    std::cout << "[SenderReceiver] Receiver thread started" << std::endl;
    
    while (running_) {
        // This thread can be used for periodic tasks or background processing
        // For now, it's mainly for coordination
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "[SenderReceiver] Receiver thread stopped" << std::endl;
}

void SenderReceiver::handleIncomingChunk(const ChunkPacket& chunk, int tunnel_id) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Add chunk to collector
    addChunkToCollector(chunk);
    
    // Update statistics
    stats_.chunks_received++;
    stats_.bytes_received += chunk.data_size;
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto latency = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    updateReceiveStats(chunk.frame_id, latency.count());
    
    std::cout << "[SenderReceiver] Received chunk " << (int)chunk.chunk_id 
              << "/" << chunk.total_chunks << " for frame " << chunk.frame_id 
              << " via tunnel " << tunnel_id << " in " << latency.count() << "μs" << std::endl;
}

void SenderReceiver::updateSendStats(uint32_t frame_id, uint64_t latency_us) {
    // Update average send latency
    if (stats_.frames_sent > 0) {
        stats_.avg_send_latency_ms = (stats_.avg_send_latency_ms * (stats_.frames_sent - 1) + 
                                     latency_us / 1000.0) / stats_.frames_sent;
    }
}

void SenderReceiver::updateReceiveStats(uint32_t frame_id, uint64_t latency_us) {
    // Update average receive latency
    if (stats_.frames_received > 0) {
        stats_.avg_receive_latency_ms = (stats_.avg_receive_latency_ms * (stats_.frames_received - 1) + 
                                        latency_us / 1000.0) / stats_.frames_received;
    }
}

// Global interface functions
void initSenderReceiver(const std::vector<int>& local_ports,
                       const std::string& target_ip,
                       const std::vector<int>& target_ports) {
    if (g_sender_receiver) {
        delete g_sender_receiver;
    }
    
    g_sender_receiver = new SenderReceiver();
    
    if (!g_sender_receiver->init(local_ports, target_ip, target_ports)) {
        throw std::runtime_error("Failed to initialize sender/receiver");
    }
    
    std::cout << "[SenderReceiver] ✅ Global sender/receiver initialized" << std::endl;
}

bool sendFrame(const std::vector<uint8_t>& frame_data, uint32_t frame_id) {
    if (!g_sender_receiver) {
        throw std::runtime_error("Sender/receiver not initialized");
    }
    
    return g_sender_receiver->sendFrame(frame_data, frame_id);
}

void setReceiveCallback(std::function<void(const std::vector<uint8_t>&, uint32_t)> callback) {
    if (g_sender_receiver) {
        g_sender_receiver->setReceiveCallback(callback);
    }
}

void startSenderReceiver() {
    if (!g_sender_receiver) {
        throw std::runtime_error("Sender/receiver not initialized");
    }
    
    g_sender_receiver->start();
}

void stopSenderReceiver() {
    if (g_sender_receiver) {
        g_sender_receiver->stop();
    }
}

void shutdownSenderReceiver() {
    if (g_sender_receiver) {
        g_sender_receiver->stop();
        delete g_sender_receiver;
        g_sender_receiver = nullptr;
    }
    
    std::cout << "[SenderReceiver] Sender/receiver shutdown complete" << std::endl;
}
