#pragma once

#include "packet_parser.hpp"  // ChunkPacket burada tanımlı
#include <vector>
#include <string>
#include <functional>

// Global socket management for both sender and receiver
bool init_global_sockets(const std::vector<int>& ports);
std::vector<int> get_global_sockets();
void close_global_sockets();

// Bidirectional I/O functions
void set_receive_callback(std::function<void(const ChunkPacket&, int)> callback);
void set_sender_callback(std::function<void(const ChunkPacket&, int)> callback);
void set_receiver_callback(std::function<void(const ChunkPacket&, int)> callback);
void start_bidirectional_receive();
void stop_bidirectional_receive();

// UDP soketlerini açar ve belirtilen yerel portlara bind eder
bool init_udp_sockets(const std::vector<int>& local_ports);

// Soketleri kapatır
void close_udp_sockets();

void set_target_addresses(const std::string& target_ip, const std::vector<int>& ports);

// Bir ChunkPacket'i hedef IP ve port'a gönder
// Non-blocking send with retry
ssize_t send_udp(const std::string& target_ip, int port, const ChunkPacket& packet);

// Multipath sending with redundancy
ssize_t send_udp_multipath(const std::string& target_ip, const std::vector<int>& ports, const ChunkPacket& packet); 