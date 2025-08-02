// src/core/udp_test_friend.cpp - Arkadaşınız için UDP Test
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <thread>
#include <atomic>
#include <vector>
#include <cstring>
#include <fcntl.h>
#include <errno.h>

class UDPTest {
private:
    int sockfd_;
    struct sockaddr_in remote_addr_;
    std::atomic<bool> running_{false};
    std::thread receive_thread_;

public:
    UDPTest(const std::string& local_ip, uint16_t local_port, 
            const std::string& remote_ip, uint16_t remote_port) {
        
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            throw std::runtime_error("Failed to create socket");
        }

        fcntl(sockfd_, F_SETFL, O_NONBLOCK);
        
        int reuse = 1;
        setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        
        struct sockaddr_in local_addr{};
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = INADDR_ANY;
        local_addr.sin_port = htons(local_port);

        if (bind(sockfd_, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
            throw std::runtime_error("Failed to bind socket to port " + std::to_string(local_port));
        }

        memset(&remote_addr_, 0, sizeof(remote_addr_));
        remote_addr_.sin_family = AF_INET;
        remote_addr_.sin_port = htons(remote_port);
        inet_pton(AF_INET, remote_ip.c_str(), &remote_addr_.sin_addr);

        std::cout << "UDP Test (Friend) initialized:" << std::endl;
        std::cout << "Local: " << local_ip << ":" << local_port << std::endl;
        std::cout << "Remote: " << remote_ip << ":" << remote_port << std::endl;
    }

    ~UDPTest() {
        stop();
        if (sockfd_ >= 0) close(sockfd_);
    }

    void start() {
        running_ = true;
        receive_thread_ = std::thread(&UDPTest::receive_loop, this);
        std::cout << "UDP Test started. Press Enter to send message, Ctrl+C to exit." << std::endl;
    }

    void stop() {
        running_ = false;
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
    }

    void send_message(const std::string& message) {
        ssize_t bytes_sent = sendto(sockfd_, message.c_str(), message.length(), 0, 
                                   (const struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
        if (bytes_sent < 0) {
            std::cerr << "Failed to send message: " << strerror(errno) << std::endl;
        } else {
            std::cout << "Sent: " << message << std::endl;
        }
    }

private:
    void receive_loop() {
        std::vector<char> buffer(1024);
        
        while (running_) {
            struct sockaddr_in src_addr;
            socklen_t addr_len = sizeof(src_addr);
            
            ssize_t bytes_read = recvfrom(sockfd_, buffer.data(), buffer.size(), 0, 
                                         (struct sockaddr*)&src_addr, &addr_len);

            if (bytes_read > 0) {
                std::string message(buffer.data(), bytes_read);
                char src_ip[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &src_addr.sin_addr, src_ip, INET_ADDRSTRLEN);
                std::cout << "Received from " << src_ip << ":" << ntohs(src_addr.sin_port) 
                          << ": " << message << std::endl;
            } else if (errno != EWOULDBLOCK && errno != EAGAIN) {
                std::cerr << "recvfrom error: " << strerror(errno) << std::endl;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
};

int main() {
    try {
        const std::string local_ip = "192.168.1.5";
        const uint16_t local_port = 50001;
        const std::string remote_ip = "192.168.1.254";
        const uint16_t remote_port = 50000;

        UDPTest test(local_ip, local_port, remote_ip, remote_port);
        test.start();

        std::string input;
        while (std::getline(std::cin, input)) {
            if (input == "quit" || input == "exit") break;
            test.send_message(input);
        }

        test.stop();
        std::cout << "UDP Test finished." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
} 