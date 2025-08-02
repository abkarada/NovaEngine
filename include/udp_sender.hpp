#pragma once

#include <vector>
#include <string>
#include <functional>
#include <atomic>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>

// NovaEngine Ultra Stream - Multi-Tunnel UDP Architecture
// Bidirectional I/O with M UDP ports for zero-latency video streaming

struct ChunkPacket {
    uint32_t frame_id;           // Frame identifier
    uint16_t chunk_id;           // Chunk index within frame
    uint16_t total_chunks;       // Total chunks in frame
    uint16_t data_size;          // Actual data size
    uint8_t data[1000];          // MTU-optimized payload
    uint64_t timestamp;          // High-precision timestamp
    uint8_t path_id;             // Which UDP tunnel used
    uint8_t priority;            // Priority level (0=normal, 1=high, 2=critical)
};

// Path statistics for adaptive scheduling
struct PathStats {
    std::string ip;
    int port;
    double rtt_ms;              // Round-trip time
    double loss_ratio;          // Packet loss ratio
    int weight;                 // Calculated weight for scheduler
    uint64_t bytes_sent;        // Total bytes sent
    uint64_t packets_sent;      // Total packets sent
    uint64_t last_activity;     // Last activity timestamp
    
    PathStats(const std::string& ip, int port, double rtt = 50.0, double loss = 0.0);
};

// Multi-tunnel UDP sender with bidirectional I/O
class UDPSender {
public:
    UDPSender();
    ~UDPSender();
    
    // Initialize M UDP tunnels
    bool initTunnels(const std::vector<int>& local_ports);
    
    // Set target addresses for each tunnel
    void setTargets(const std::string& target_ip, const std::vector<int>& target_ports);
    
    // Send chunk through weighted scheduler
    ssize_t sendChunk(const ChunkPacket& packet);
    
    // Send chunk through specific tunnel
    ssize_t sendChunkViaTunnel(const ChunkPacket& packet, int tunnel_id);
    
    // Start bidirectional receive threads
    void startBidirectionalReceive(std::function<void(const ChunkPacket&, int)> callback);
    
    // Stop receive threads
    void stopBidirectionalReceive();
    
    // Update path statistics
    void updatePathStats(const std::vector<PathStats>& stats);
    
    // Get current path statistics
    std::vector<PathStats> getPathStats() const;
    
    // Close all tunnels
    void closeTunnels();

private:
    std::vector<int> sockets_;
    std::vector<sockaddr_in> target_addrs_;
    std::vector<PathStats> path_stats_;
    std::vector<std::thread> receive_threads_;
    std::atomic<bool> running_;
    std::function<void(const ChunkPacket&, int)> receive_callback_;
    
    // Bidirectional receive thread function
    void receiveThread(int socket_id, int port);
    
    // Set socket options for optimal performance
    bool configureSocket(int sock);
};

// Global interface functions
bool initGlobalSockets(const std::vector<int>& ports);
void setReceiveCallback(std::function<void(const ChunkPacket&, int)> callback);
std::vector<int> getGlobalSockets();
void closeGlobalSockets();

