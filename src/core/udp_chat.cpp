// src/core/udp_chat.cpp - Manuel IP/Port Girişli UDP Chat
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
#include <regex>

class UDPChat {
private:
    int sockfd_;
    struct sockaddr_in remote_addr_;
    std::atomic<bool> running_{false};
    std::thread receive_thread_;
    std::string local_ip_;
    uint16_t local_port_;
    std::string remote_ip_;
    uint16_t remote_port_;

public:
    UDPChat(const std::string& local_ip, uint16_t local_port, 
            const std::string& remote_ip, uint16_t remote_port)
        : local_ip_(local_ip), local_port_(local_port),
          remote_ip_(remote_ip), remote_port_(remote_port) {
        
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

        std::cout << "\n=== UDP Chat Başlatıldı ===" << std::endl;
        std::cout << "Local: " << local_ip << ":" << local_port << std::endl;
        std::cout << "Remote: " << remote_ip << ":" << remote_port << std::endl;
        std::cout << "============================\n" << std::endl;
    }

    ~UDPChat() {
        stop();
        if (sockfd_ >= 0) close(sockfd_);
    }

    void start() {
        running_ = true;
        receive_thread_ = std::thread(&UDPChat::receive_loop, this);
        std::cout << "Chat başlatıldı! Mesaj yazın (çıkmak için 'quit' yazın):\n" << std::endl;
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
            std::cerr << "Mesaj gönderilemedi: " << strerror(errno) << std::endl;
        } else {
            std::cout << "Siz: " << message << std::endl;
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
                std::cout << "Karşı taraf (" << src_ip << ":" << ntohs(src_addr.sin_port) 
                          << "): " << message << std::endl;
            } else if (errno != EWOULDBLOCK && errno != EAGAIN) {
                std::cerr << "Alma hatası: " << strerror(errno) << std::endl;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
};

bool is_valid_ip(const std::string& ip) {
    std::regex ip_regex("^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$");
    return std::regex_match(ip, ip_regex);
}

bool is_valid_port(uint16_t port) {
    return port > 0 && port < 65536;
}

uint16_t get_port_input(const std::string& prompt) {
    uint16_t port;
    while (true) {
        std::cout << prompt;
        std::string input;
        std::getline(std::cin, input);
        
        try {
            port = std::stoi(input);
            if (is_valid_port(port)) {
                return port;
            } else {
                std::cout << "Hata: Port 1-65535 arasında olmalıdır.\n";
            }
        } catch (const std::exception&) {
            std::cout << "Hata: Geçerli bir sayı girin.\n";
        }
    }
}

std::string get_ip_input(const std::string& prompt) {
    std::string ip;
    while (true) {
        std::cout << prompt;
        std::getline(std::cin, ip);
        
        if (is_valid_ip(ip)) {
            return ip;
        } else {
            std::cout << "Hata: Geçerli bir IP adresi girin (örn: 192.168.1.100)\n";
        }
    }
}

int main() {
    try {
        std::cout << "=== Nova Engine V3 - UDP Chat ===" << std::endl;
        std::cout << "Manuel IP ve Port Girişi\n" << std::endl;

        // Kendi IP'nizi girin
        std::string local_ip = get_ip_input("Kendi IP adresinizi girin: ");
        uint16_t local_port = get_port_input("Kendi port numaranızı girin (1-65535): ");

        // Karşı tarafın IP'sini girin
        std::string remote_ip = get_ip_input("Karşı tarafın IP adresini girin: ");
        uint16_t remote_port = get_port_input("Karşı tarafın port numarasını girin (1-65535): ");

        std::cout << "\n=== Bağlantı Bilgileri ===" << std::endl;
        std::cout << "Sizin IP:Port = " << local_ip << ":" << local_port << std::endl;
        std::cout << "Karşı taraf IP:Port = " << remote_ip << ":" << remote_port << std::endl;
        std::cout << "==========================\n" << std::endl;

        UDPChat chat(local_ip, local_port, remote_ip, remote_port);
        chat.start();

        std::string input;
        while (std::getline(std::cin, input)) {
            if (input == "quit" || input == "exit" || input == "q") {
                break;
            }
            if (!input.empty()) {
                chat.send_message(input);
            }
        }

        chat.stop();
        std::cout << "\nChat sonlandırıldı." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Hata: " << e.what() << std::endl;
        return 1;
    }

    return 0;
} 