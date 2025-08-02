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

static std::vector<int> udp_sockets;
static std::vector<std::string> target_ips;
static std::vector<int> target_ports;
static std::vector<sockaddr_in> target_addrs;

// Global socket management for both sender and receiver
static std::vector<int> global_sockets;
static bool global_sockets_initialized = false;

// Bidirectional I/O support
static std::function<void(const ChunkPacket&, int)> receive_callback = nullptr;
static std::function<void(const ChunkPacket&, int)> sender_callback = nullptr;
static std::function<void(const ChunkPacket&, int)> receiver_callback = nullptr;
static std::atomic<bool> bidirectional_mode = false;
static std::atomic<bool> stop_receive_thread = false;
static std::thread receive_thread;

// Initialize global sockets that can be used by both sender and receiver
bool init_global_sockets(const std::vector<int>& ports) {
    if (global_sockets_initialized) {
        std::cout << "[global_sockets] Already initialized with " << global_sockets.size() << " sockets" << std::endl;
        return true; // Already initialized
    }
    
    global_sockets.clear();
    
    for (int port : ports) {
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            perror("Socket creation failed");
            return false;
        }
        
        // Set socket options for port reuse (both send and receive on same port)
        int optval = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
        
        // Enable SO_REUSEPORT for multiple processes to bind to same port
        #ifdef SO_REUSEPORT
        setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
        #endif
        
        // Set send buffer size for better throughput
        int sendbuf = 65536;
        setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sendbuf, sizeof(sendbuf));
        
        // Set receive buffer size
        int recvbuf = 65536;
        setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &recvbuf, sizeof(recvbuf));
        
        // Set non-blocking mode
        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);
        
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = INADDR_ANY;
        if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            perror("bind failed");
            close(sock);
            return false;
        }
        
        global_sockets.push_back(sock);
        std::cout << "[global_sockets] Socket bound to port " << port << std::endl;
    }
    
    global_sockets_initialized = true;
    std::cout << "[global_sockets] ✅ " << global_sockets.size() << " UDP sockets prepared for bidirectional I/O.\n";
    return true;
}

// Set receive callback for bidirectional mode
void set_receive_callback(std::function<void(const ChunkPacket&, int)> callback) {
    receive_callback = callback;
}

void set_sender_callback(std::function<void(const ChunkPacket&, int)> callback) {
    sender_callback = callback;
}

void set_receiver_callback(std::function<void(const ChunkPacket&, int)> callback) {
    receiver_callback = callback;
}

// Start bidirectional receive thread
void start_bidirectional_receive() {
    if (bidirectional_mode.exchange(true)) {
        std::cout << "[bidirectional] Already in bidirectional mode" << std::endl;
        return;
    }
    
    stop_receive_thread = false;
    receive_thread = std::thread([&]() {
        constexpr int MAX_BUFFER = 1500;
        uint8_t buf[MAX_BUFFER];
        
        std::cout << "[bidirectional] Starting receive thread for " << global_sockets.size() << " sockets" << std::endl;
        
        while (!stop_receive_thread) {
            for (size_t i = 0; i < global_sockets.size(); ++i) {
                int sock = global_sockets[i];
                
                ssize_t len = recv(sock, buf, sizeof(buf), MSG_DONTWAIT);
                if (len >= 12) { // Minimum size for timestamp
                    std::cout << "[bidirectional] Received " << len << " bytes on socket " << i << std::endl;
                    try {
                        auto pkt = parse_packet(buf, len);
                        
                        // Calculate RTT if timestamp is present
                        if (pkt.timestamp > 0) {
                            auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::steady_clock::now().time_since_epoch()).count();
                            auto rtt_us = now_us - pkt.timestamp;
                            auto rtt_ms = rtt_us / 1000.0;
                            
                            // Log RTT occasionally
                            static int rtt_log_counter = 0;
                            if (++rtt_log_counter % 100 == 0) {
                                std::cout << "[bidirectional] Socket " << i << " RTT: " << rtt_ms << "ms" << std::endl;
                            }
                        }
                        
                        // Call receive callback
                        if (receive_callback) {
                            std::cout << "[bidirectional] Calling receive callback for socket " << i << std::endl;
                            receive_callback(pkt, i);
                        } else {
                            std::cout << "[bidirectional] No receive callback set!" << std::endl;
                        }
                        
                        // Call sender callback if set
                        if (sender_callback) {
                            sender_callback(pkt, i);
                        }
                        
                        // Call receiver callback if set
                        if (receiver_callback) {
                            receiver_callback(pkt, i);
                        }
                        
                        std::cout << "[bidirectional] Received " << len << " bytes on socket " << i 
                                 << " (frame " << pkt.frame_id << ", chunk " << (int)pkt.chunk_id << ")" << std::endl;
                        
                    } catch (const std::exception& e) {
                        std::cerr << "[bidirectional] Parse error on socket " << i << ": " << e.what() << std::endl;
                    }
                } else if (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                    std::cerr << "[bidirectional] Recv error on socket " << i << ": " << strerror(errno) << std::endl;
                }
            }
            
            // Small delay to prevent busy waiting
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        
        std::cout << "[bidirectional] Receive thread stopped" << std::endl;
    });
}

// Stop bidirectional receive thread
void stop_bidirectional_receive() {
    if (!bidirectional_mode.exchange(false)) {
        return;
    }
    
    stop_receive_thread = true;
    if (receive_thread.joinable()) {
        receive_thread.join();
    }
}

// Get global sockets for receiver
std::vector<int> get_global_sockets() {
    return global_sockets;
}

// Close global sockets
void close_global_sockets() {
    stop_bidirectional_receive();
    
    for (int sock : global_sockets) {
        close(sock);
    }
    global_sockets.clear();
    global_sockets_initialized = false;
    std::cout << "[global_sockets] All global sockets closed.\n";
}

bool init_udp_sockets(const std::vector<int>& local_ports) {
    // Use global socket management
    return init_global_sockets(local_ports);
}

void close_udp_sockets() {
    // Don't close global sockets here, they might be used by receiver
    // Only clear the sender-specific data
    udp_sockets.clear();
    target_addrs.clear();
    std::cout << "[udp_sender] Sender sockets cleared.\n";
}

// Pre-compute target addresses for zero-copy optimization
void set_target_addresses(const std::string& target_ip, const std::vector<int>& ports) {
    target_addrs.clear();
    target_ips.clear();
    target_ports.clear();
    
    for (int port : ports) {
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        inet_pton(AF_INET, target_ip.c_str(), &addr.sin_addr);
        target_addrs.push_back(addr);
        target_ips.push_back(target_ip);
        target_ports.push_back(port);
    }
}

ssize_t send_udp(const std::string& target_ip, int port, const ChunkPacket& packet) {
    if (!global_sockets_initialized || global_sockets.empty()) return -1;

    // Find the target address
    size_t addr_index = 0;
    bool found = false;
    for (size_t i = 0; i < target_ports.size(); ++i) {
        if (target_ports[i] == port && target_ips[i] == target_ip) {
            addr_index = i;
            found = true;
            break;
        }
    }
    
    if (!found) {
        // Add new target address
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        inet_pton(AF_INET, target_ip.c_str(), &addr.sin_addr);
        target_addrs.push_back(addr);
        target_ips.push_back(target_ip);
        target_ports.push_back(port);
        addr_index = target_addrs.size() - 1;
    }

    // Select socket using round-robin for load balancing
    static size_t socket_index = 0;
    int sock = global_sockets[socket_index % global_sockets.size()];
    socket_index++;

    // Serialize packet
    std::vector<uint8_t> buffer = serialize_packet(packet);

    // Non-blocking send with retry
    ssize_t sent = 0;
    int retries = 0;
    const int max_retries = 3;
    
    while (retries < max_retries) {
        sent = sendto(sock, buffer.data(), buffer.size(), MSG_DONTWAIT,
                      reinterpret_cast<sockaddr*>(&target_addrs[addr_index]), 
                      sizeof(sockaddr_in));
        
        if (sent >= 0) {
            std::cout << "[bidirectional] Sent " << sent << " bytes to " << target_ip << ":" << port 
                     << " via socket " << (socket_index - 1) % global_sockets.size() << std::endl;
            break; // Success
        }
        
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // Buffer full, try again after a short delay
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            retries++;
        } else {
            // Other error, don't retry
            perror("[udp_sender] Send error");
            break;
        }
    }

    if (sent < 0 && retries >= max_retries) {
        std::cerr << "[udp_sender] Failed to send after " << max_retries << " retries" << std::endl;
    }
    
    return sent;
}

// Enhanced send with multiple paths for redundancy
ssize_t send_udp_multipath(const std::string& target_ip, const std::vector<int>& ports, 
                          const ChunkPacket& packet) {
    if (!global_sockets_initialized || global_sockets.empty()) return -1;

    ssize_t total_sent = 0;
    std::vector<ssize_t> sent_per_path;
    
    // Send to all paths for redundancy
    for (int port : ports) {
        ssize_t sent = send_udp(target_ip, port, packet);
        sent_per_path.push_back(sent);
        if (sent > 0) total_sent += sent;
    }
    
    // Log if some paths failed
    int failed_paths = 0;
    for (ssize_t sent : sent_per_path) {
        if (sent < 0) failed_paths++;
    }
    
    if (failed_paths > 0 && failed_paths < static_cast<int>(ports.size())) {
        std::cout << "[udp_sender] Sent via " << (ports.size() - failed_paths) 
                 << "/" << ports.size() << " paths" << std::endl;
    }
    
    return total_sent;
}
