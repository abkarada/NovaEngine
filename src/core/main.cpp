// src/core/main.cpp - Görüntülü Video Chat
#include <iostream>
#include <opencv2/opencv.hpp>
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

class VideoChat {
private:
    int sockfd_;
    struct sockaddr_in remote_addr_;
    std::atomic<bool> running_{false};
    std::thread receive_thread_;
    std::string local_ip_;
    uint16_t local_port_;
    std::string remote_ip_;
    uint16_t remote_port_;
    cv::VideoCapture cap_;
    cv::VideoWriter writer_;

public:
    VideoChat(const std::string& local_ip, uint16_t local_port, 
              const std::string& remote_ip, uint16_t remote_port)
        : local_ip_(local_ip), local_port_(local_port),
          remote_ip_(remote_ip), remote_port_(remote_port) {
        
        // Initialize camera
        cap_.open(0);
        if (!cap_.isOpened()) {
            throw std::runtime_error("Kamera açılamadı!");
        }
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap_.set(cv::CAP_PROP_FPS, 30);

        // Initialize UDP socket
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            throw std::runtime_error("Socket oluşturulamadı");
        }

        fcntl(sockfd_, F_SETFL, O_NONBLOCK);
        
        int reuse = 1;
        setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        
        struct sockaddr_in local_addr{};
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = INADDR_ANY;
        local_addr.sin_port = htons(local_port);

        if (bind(sockfd_, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
            throw std::runtime_error("Port " + std::to_string(local_port) + " bağlanamadı");
        }

        memset(&remote_addr_, 0, sizeof(remote_addr_));
        remote_addr_.sin_family = AF_INET;
        remote_addr_.sin_port = htons(remote_port);
        inet_pton(AF_INET, remote_ip.c_str(), &remote_addr_.sin_addr);

        std::cout << "\n=== Video Chat Başlatıldı ===" << std::endl;
        std::cout << "Local: " << local_ip << ":" << local_port << std::endl;
        std::cout << "Remote: " << remote_ip << ":" << remote_port << std::endl;
        std::cout << "============================\n" << std::endl;
    }

    ~VideoChat() {
        stop();
        if (sockfd_ >= 0) close(sockfd_);
        cap_.release();
        cv::destroyAllWindows();
    }

    void start() {
        running_ = true;
        receive_thread_ = std::thread(&VideoChat::receive_loop, this);
        std::cout << "Video chat başlatıldı! ESC tuşu ile çıkın.\n" << std::endl;
        
        // Main video loop
        cv::Mat frame;
        while (running_) {
            cap_ >> frame;
            if (frame.empty()) continue;

            // Display local video
            cv::imshow("Sizin Görüntünüz", frame);

            // Send frame data
            send_frame(frame);

            char key = cv::waitKey(1);
            if (key == 27) { // ESC key
                break;
            }
        }
    }

    void stop() {
        running_ = false;
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
    }

private:
    void send_frame(const cv::Mat& frame) {
        // Convert frame to JPEG
        std::vector<uchar> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
        cv::imencode(".jpg", frame, buffer, params);

        // Send frame data
        ssize_t bytes_sent = sendto(sockfd_, buffer.data(), buffer.size(), 0, 
                                   (const struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
        if (bytes_sent < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
            std::cerr << "Frame gönderilemedi: " << strerror(errno) << std::endl;
        }
    }

    void receive_loop() {
        std::vector<uchar> buffer(65536); // Large buffer for video frames
        
        while (running_) {
            struct sockaddr_in src_addr;
            socklen_t addr_len = sizeof(src_addr);
            
            ssize_t bytes_read = recvfrom(sockfd_, buffer.data(), buffer.size(), 0, 
                                         (struct sockaddr*)&src_addr, &addr_len);

            if (bytes_read > 0) {
                // Decode received frame
                std::vector<uchar> frame_data(buffer.begin(), buffer.begin() + bytes_read);
                cv::Mat received_frame = cv::imdecode(frame_data, cv::IMREAD_COLOR);
                
                if (!received_frame.empty()) {
                    cv::imshow("Karşı Taraf", received_frame);
                    cv::waitKey(1);
                }
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
        std::cout << "=== Nova Engine V3 - Video Chat ===" << std::endl;
        std::cout << "Görüntülü İletişim\n" << std::endl;

        // Kendi IP'nizi girin
        std::string local_ip = get_ip_input("Kendi IP adresinizi girin: ");
        uint16_t local_port = get_port_input("Kendi port numaranızı girin (1-65535): ");

        // Karşı tarafın IP'sini girin
        std::string remote_ip = get_ip_input("Karşı tarafın IP adresini girin: ");
        uint16_t remote_port = get_port_input("Karşı tarafın port numarasını girin (1-65535): ");

        std::cout << "\n=== Video Chat Bağlantısı ===" << std::endl;
        std::cout << "Sizin IP:Port = " << local_ip << ":" << local_port << std::endl;
        std::cout << "Karşı taraf IP:Port = " << remote_ip << ":" << remote_port << std::endl;
        std::cout << "==============================\n" << std::endl;

        VideoChat chat(local_ip, local_port, remote_ip, remote_port);
        chat.start();

        std::cout << "\nVideo chat sonlandırıldı." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Hata: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}