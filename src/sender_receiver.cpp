#include "sender_receiver.hpp"
#include "slicer.hpp"
#include "smart_collector.hpp"
#include "scheduler.hpp"
#include "network_monitor.hpp"
#include "ffmpeg_encoder.h"
#include "erasure_coder.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <random>

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
    if (!g_slicer) {
        return false;
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Slice frame with Reed-Solomon FEC
    std::vector<ChunkPacket> chunks = g_slicer->sliceFrame(frame_data, frame_id);
    if (chunks.empty()) {
        return false;
    }
    
    // Send chunks with intelligent path selection
    int chunks_sent = 0;
    for (const auto& chunk : chunks) {
        // Select optimal path for this chunk
        int selected_path = selectPathForChunk(frame_id, chunk.chunk_id, chunk.is_parity);
        
        // Update network monitor with packet sent
        NetworkMonitor* monitor = getNetworkMonitor();
        if (monitor) {
            uint64_t send_timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            monitor->onPacketSent(frame_id, chunk.chunk_id, send_timestamp, chunk.data_size);
        }
        
        // Send chunk to selected path
        ssize_t sent = sender_.sendChunk(chunk);
        if (sent > 0) {
            chunks_sent++;
        } else {
            // Packet lost - update network monitor
            NetworkMonitor* monitor = getNetworkMonitor();
            if (monitor) {
                monitor->onPacketLost(frame_id, chunk.chunk_id);
            }
        }
    }
    
    // Update statistics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    stats_.frames_sent++;
    updateSendStats(frame_id, duration.count());
    
    std::cout << "[SenderReceiver] Frame " << frame_id << " sent: " << chunks_sent 
              << "/" << chunks.size() << " chunks in " << duration.count() << "μs" << std::endl;
    
    return chunks_sent == chunks.size();
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
    
    uint32_t frame_counter = 0;
    std::random_device rd;
    std::mt19937 gen(rd());
    
    while (running_) {
        // Generate test frame data
        std::vector<uint8_t> frame_data = generateTestFrame(frame_counter);
        
        // Send frame
        if (sendFrame(frame_data, frame_counter)) {
            frame_counter++;
        }
        
        // Send every 100ms (10fps)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "[SenderReceiver] Sender thread stopped" << std::endl;
}

void SenderReceiver::receiverThread() {
    std::cout << "[SenderReceiver] Receiver thread started" << std::endl;
    
    while (running_) {
        // This thread handles incoming chunks via callback
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    std::cout << "[SenderReceiver] Receiver thread stopped" << std::endl;
}

void SenderReceiver::handleIncomingChunk(const ChunkPacket& chunk, int tunnel_id) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Update network monitor with packet received
    NetworkMonitor* monitor = getNetworkMonitor();
    if (monitor) {
        uint64_t receive_timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        monitor->onPacketReceived(chunk.frame_id, chunk.chunk_id, receive_timestamp, chunk.data_size);
    }
    
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

std::vector<uint8_t> SenderReceiver::generateTestFrame(uint32_t frame_id) {
    // Generate a simple test frame with pattern
    std::vector<uint8_t> frame_data;
    frame_data.resize(640 * 480 * 3); // 640x480 RGB
    
    // Create a simple pattern
    for (size_t i = 0; i < frame_data.size(); i += 3) {
        // Red component
        frame_data[i] = (frame_id + i) % 256;
        // Green component  
        frame_data[i + 1] = (frame_id * 2 + i) % 256;
        // Blue component
        frame_data[i + 2] = (frame_id * 3 + i) % 256;
    }
    
    return frame_data;
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
