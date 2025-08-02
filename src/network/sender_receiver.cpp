#include "sender_receiver.h"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>

SenderReceiver::SenderReceiver(
    uint16_t port,
    const std::string& remote_ip,
    uint16_t remote_port,
    std::function<void(ChunkPtr)> on_chunk_received
) : on_chunk_received_(on_chunk_received) {
    
    // Create UDP socket
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
        throw std::runtime_error("Failed to create socket");
    }
    
    // Set up remote address
    memset(&remote_addr_, 0, sizeof(remote_addr_));
    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_port = htons(remote_port);
    remote_addr_.sin_addr.s_addr = inet_addr(remote_ip.c_str());
    
    std::cout << "SenderReceiver initialized for " << remote_ip << ":" << remote_port << std::endl;
}

SenderReceiver::~SenderReceiver() {
    stop();
    if (sockfd_ >= 0) {
        close(sockfd_);
    }
    std::cout << "SenderReceiver destroyed" << std::endl;
}

void SenderReceiver::start() {
    running_ = true;
    io_thread_ = std::thread(&SenderReceiver::io_loop, this);
    std::cout << "SenderReceiver started" << std::endl;
}

void SenderReceiver::stop() {
    running_ = false;
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
    std::cout << "SenderReceiver stopped" << std::endl;
}

void SenderReceiver::sendChunk(ChunkPtr chunk) {
    if (chunk) {
        auto data = chunk->serialize();
        sendto(sockfd_, data.data(), data.size(), 0,
               (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
    }
}

void SenderReceiver::io_loop() {
    std::vector<uint8_t> buffer(2048);
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    
    while (running_) {
        ssize_t bytes_received = recvfrom(sockfd_, buffer.data(), buffer.size(), 0,
                                         (struct sockaddr*)&client_addr, &client_len);
        if (bytes_received > 0) {
            buffer.resize(bytes_received);
            auto chunk = std::make_shared<Chunk>(Chunk::deserialize(buffer));
            if (on_chunk_received_) {
                on_chunk_received_(chunk);
            }
        }
    }
} 