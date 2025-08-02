// sender_receiver.h
#pragma once

#include "../media/slicer.h"
#include "../transport/smart_collector.h"
#include "../common/concurrent_queue.h" // A thread-safe queue implementation
#include <string>
#include <thread>
#include <atomic>
#include <memory>
#include <functional>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

class SenderReceiver {
public:
    SenderReceiver(
        uint16_t port,
        const std::string& remote_ip,
        uint16_t remote_port,
        std::function<void(ChunkPtr)> on_chunk_received
    );
    ~SenderReceiver();

    void start();
    void stop();
    void sendChunk(ChunkPtr chunk);

private:
    void io_loop();
    
    int sockfd_;
    struct sockaddr_in remote_addr_;
    
    std::thread io_thread_;
    std::atomic<bool> running_{false};
    
    // A thread-safe queue is required here
    ConcurrentQueue<ChunkPtr> send_queue_;
    std::function<void(ChunkPtr)> on_chunk_received_;
};