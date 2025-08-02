#pragma once

#include "udp_sender.hpp"
#include <functional>
#include <atomic>
#include <thread>

// NovaEngine Ultra Stream - Zero-Copy Sender/Receiver
// Ultra-low latency bidirectional I/O with std::move optimization

class SenderReceiver {
public:
    SenderReceiver();
    ~SenderReceiver();
    
    // Initialize with M UDP tunnels
    bool init(const std::vector<int>& local_ports, 
              const std::string& target_ip, 
              const std::vector<int>& target_ports);
    
    // Send frame data through multi-tunnel
    bool sendFrame(const std::vector<uint8_t>& frame_data, uint32_t frame_id);
    
    // Set receive callback for incoming frames
    void setReceiveCallback(std::function<void(const std::vector<uint8_t>&, uint32_t)> callback);
    
    // Start sender and receiver threads
    void start();
    
    // Stop all threads
    void stop();
    
    // Get sender/receiver statistics
    struct SRStats {
        uint32_t frames_sent;
        uint32_t frames_received;
        uint32_t chunks_sent;
        uint32_t chunks_received;
        uint64_t bytes_sent;
        uint64_t bytes_received;
        double avg_send_latency_ms;
        double avg_receive_latency_ms;
        double throughput_mbps;
    };
    
    SRStats getStats() const;
    
    // Check if running
    bool isRunning() const { return running_; }

private:
    UDPSender sender_;
    std::atomic<bool> running_;
    std::thread sender_thread_;
    std::thread receiver_thread_;
    std::function<void(const std::vector<uint8_t>&, uint32_t)> receive_callback_;
    SRStats stats_;
    
    // Sender thread function
    void senderThread();
    
    // Receiver thread function
    void receiverThread();
    
    // Handle incoming chunk
    void handleIncomingChunk(const ChunkPacket& chunk, int tunnel_id);
    
    // Update statistics
    void updateSendStats(uint32_t frame_id, uint64_t latency_us);
    void updateReceiveStats(uint32_t frame_id, uint64_t latency_us);
    
    // Generate test frame data
    std::vector<uint8_t> generateTestFrame(uint32_t frame_id);
};

// Global sender/receiver interface
extern SenderReceiver* g_sender_receiver;

void initSenderReceiver(const std::vector<int>& local_ports,
                       const std::string& target_ip,
                       const std::vector<int>& target_ports);
bool sendFrame(const std::vector<uint8_t>& frame_data, uint32_t frame_id);
void setReceiveCallback(std::function<void(const std::vector<uint8_t>&, uint32_t)> callback);
void startSenderReceiver();
void stopSenderReceiver();
void shutdownSenderReceiver();
