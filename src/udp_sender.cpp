#include "udp_sender.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <cstdint>
#include <fcntl.h>
#include <errno.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <functional>
#include <atomic>

// NovaEngine Ultra Stream - Multi-Tunnel UDP Implementation
// Bidirectional I/O with M UDP ports for zero-latency video streaming

// Global socket management
static std::vector<int> global_sockets;
static bool global_sockets_initialized = false;
static std::function<void(const ChunkPacket&, int)> global_receive_callback = nullptr;

// PathStats implementation - constructor is already defined in header

// UDPSender implementation
UDPSender::UDPSender() : running_(false) {
}

UDPSender::~UDPSender() {
    closeTunnels();
}

bool UDPSender::initTunnels(const std::vector<int>& local_ports) {
    sockets_.clear();
    target_addrs_.clear();
    path_stats_.clear();
    
    for (int port : local_ports) {
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            perror("Socket creation failed");
            return false;
        }
        
        if (!configureSocket(sock)) {
            close(sock);
            return false;
        }
        
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = INADDR_ANY;
        
        if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            perror("bind failed");
            close(sock);
            return false;
        }
        
        sockets_.push_back(sock);
        path_stats_.emplace_back("127.0.0.1", port, 50.0, 0.0);
        
        std::cout << "[UDPSender] Tunnel initialized on port " << port << std::endl;
    }
    
    std::cout << "[UDPSender] ✅ " << sockets_.size() << " UDP tunnels ready for bidirectional I/O" << std::endl;
    return true;
}

void UDPSender::setTargets(const std::string& target_ip, const std::vector<int>& target_ports) {
    target_addrs_.clear();
    
    for (int port : target_ports) {
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = inet_addr(target_ip.c_str());
        target_addrs_.push_back(addr);
    }
    
    std::cout << "[UDPSender] Targets set: " << target_ip << ":" << target_ports[0] << std::endl;
}

ssize_t UDPSender::sendChunk(const ChunkPacket& packet) {
    if (sockets_.empty()) return -1;
    
    // Use weighted round-robin for path selection
    static int current_path = 0;
    int selected_path = current_path % sockets_.size();
    current_path++;
    
    return sendChunkViaTunnel(packet, selected_path);
}

ssize_t UDPSender::sendChunkViaTunnel(const ChunkPacket& packet, int tunnel_id) {
    if (tunnel_id < 0 || tunnel_id >= sockets_.size() || tunnel_id >= target_addrs_.size()) {
        return -1;
    }
    
    int sock = sockets_[tunnel_id];
    const sockaddr_in& target = target_addrs_[tunnel_id];
    
    // Prepare packet with timestamp
    ChunkPacket send_packet = packet;
    send_packet.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    send_packet.path_id = tunnel_id;
    
    ssize_t sent = sendto(sock, &send_packet, sizeof(send_packet), 0,
                         reinterpret_cast<const sockaddr*>(&target), sizeof(target));
    
    if (sent > 0) {
        // Update statistics
        path_stats_[tunnel_id].bytes_sent += sent;
        path_stats_[tunnel_id].packets_sent++;
        path_stats_[tunnel_id].last_activity = send_packet.timestamp;
    } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // Non-blocking socket is full, retry or drop
        std::cout << "[UDPSender] Socket " << tunnel_id << " buffer full, dropping packet" << std::endl;
    }
    
    return sent;
}

void UDPSender::startBidirectionalReceive(std::function<void(const ChunkPacket&, int)> callback) {
    if (running_) return;
    
    receive_callback_ = callback;
    running_ = true;
    receive_threads_.clear();
    
    for (size_t i = 0; i < sockets_.size(); ++i) {
        receive_threads_.emplace_back(&UDPSender::receiveThread, this, i, path_stats_[i].port);
    }
    
    std::cout << "[UDPSender] Started " << receive_threads_.size() << " bidirectional receive threads" << std::endl;
}

void UDPSender::stopBidirectionalReceive() {
    running_ = false;
    
    for (auto& thread : receive_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    receive_threads_.clear();
}

void UDPSender::receiveThread(int socket_id, int port) {
    int sock = sockets_[socket_id];
    ChunkPacket packet;
    sockaddr_in sender_addr{};
    socklen_t addr_len = sizeof(sender_addr);
    
    std::cout << "[UDPSender] Receive thread started for socket " << socket_id << " (port " << port << ")" << std::endl;
    
    while (running_) {
        ssize_t received = recvfrom(sock, &packet, sizeof(packet), 0,
                                  reinterpret_cast<sockaddr*>(&sender_addr), &addr_len);
        
        if (received > 0 && receive_callback_) {
            packet.path_id = socket_id;
            receive_callback_(packet, socket_id);
        } else if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            perror("recvfrom failed");
        }
        
        // Small sleep to prevent busy waiting
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

bool UDPSender::configureSocket(int sock) {
    // Set socket options for optimal performance
    int optval = 1;
    
    // Enable port reuse
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    
    #ifdef SO_REUSEPORT
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
    #endif
    
    // Set buffer sizes
    int sendbuf = 65536;
    int recvbuf = 65536;
    setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sendbuf, sizeof(sendbuf));
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &recvbuf, sizeof(recvbuf));
    
    // Set non-blocking mode
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    
    return true;
}

void UDPSender::updatePathStats(const std::vector<PathStats>& stats) {
    path_stats_ = stats;
}

std::vector<PathStats> UDPSender::getPathStats() const {
    return path_stats_;
}

void UDPSender::closeTunnels() {
    stopBidirectionalReceive();
    
    for (int sock : sockets_) {
        close(sock);
    }
    sockets_.clear();
    target_addrs_.clear();
    path_stats_.clear();
}

// Global interface functions
bool initGlobalSockets(const std::vector<int>& ports) {
    if (global_sockets_initialized) {
        return true;
    }
    
    global_sockets.clear();
    
    for (int port : ports) {
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            perror("Global socket creation failed");
            return false;
        }
        
        int optval = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
        
        #ifdef SO_REUSEPORT
        setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
        #endif
        
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = INADDR_ANY;
        
        if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            perror("Global bind failed");
            close(sock);
            return false;
        }
        
        global_sockets.push_back(sock);
    }
    
    global_sockets_initialized = true;
    std::cout << "[GlobalSockets] ✅ " << global_sockets.size() << " global UDP sockets ready" << std::endl;
    return true;
}

void setReceiveCallback(std::function<void(const ChunkPacket&, int)> callback) {
    global_receive_callback = callback;
}

std::vector<int> getGlobalSockets() {
    return global_sockets;
}

void closeGlobalSockets() {
    for (int sock : global_sockets) {
        close(sock);
    }
    global_sockets.clear();
    global_sockets_initialized = false;
}
