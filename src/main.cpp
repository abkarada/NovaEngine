#include "sender_receiver.hpp"
#include "slicer.hpp"
#include "smart_collector.hpp"
#include "scheduler.hpp"
#include "network_monitor.hpp"
#include "ffmpeg_encoder.h"
#include "erasure_coder.hpp"
#include "decode_and_display.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>
#include <atomic>
#include <random>

// NovaEngine Ultra Stream - P2P Video Chat
// Two clients streaming video to each other

// Global running flag
std::atomic<bool> g_running{true};

// Global video components
cv::VideoCapture g_camera;
H264Decoder g_decoder;
bool g_display_enabled = true;

void signalHandler(int signum) {
    std::cout << "\n[NovaEngine] Shutting down..." << std::endl;
    g_running = false;
}

// Camera capture thread - sends my video to peer
void cameraCaptureThread() {
    std::cout << "[Camera] Starting camera capture for P2P streaming..." << std::endl;
    
    // Open camera (0 = default camera)
    g_camera.open(0);
    if (!g_camera.isOpened()) {
        std::cerr << "[Camera] Failed to open camera! Trying alternative..." << std::endl;
        // Try alternative camera
        g_camera.open(1);
        if (!g_camera.isOpened()) {
            std::cerr << "[Camera] No camera found! Exiting..." << std::endl;
            return;
        }
    }
    
    // Set camera properties
    g_camera.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    g_camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    g_camera.set(cv::CAP_PROP_FPS, 30);
    g_camera.set(cv::CAP_PROP_BUFFERSIZE, 1); // Minimize latency
    
    std::cout << "[Camera] Camera opened: " 
              << g_camera.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
              << g_camera.get(cv::CAP_PROP_FRAME_HEIGHT) << "@"
              << g_camera.get(cv::CAP_PROP_FPS) << "fps" << std::endl;
    
    uint32_t frame_counter = 0;
    
    while (g_running) {
        cv::Mat frame;
        if (g_camera.read(frame)) {
            // Check if frame is valid
            if (frame.empty()) {
                std::cerr << "[Camera] Empty frame received!" << std::endl;
                continue;
            }
            
            // Force resize to 640x640 for encoder compatibility
            cv::Mat resized_frame;
            cv::resize(frame, resized_frame, cv::Size(640, 640));
            
            // Add timestamp text to frame
            cv::putText(resized_frame, "NovaEngine P2P - Local", cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            cv::putText(resized_frame, "Frame: " + std::to_string(frame_counter), cv::Point(10, 60), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            
            // Encode and send frame to peer
            std::vector<uint8_t> encoded_data;
            if (encodeFrame(resized_frame, encoded_data)) {
                if (g_sender_receiver) {
                    g_sender_receiver->sendFrame(encoded_data, frame_counter);
                    std::cout << "[P2P] Sent frame " << frame_counter << " to peer (" << encoded_data.size() << " bytes)" << std::endl;
                }
            } else {
                std::cerr << "[Camera] Failed to encode frame " << frame_counter << std::endl;
            }
            
            frame_counter++;
        } else {
            std::cerr << "[Camera] Failed to read frame!" << std::endl;
        }
        
        // Capture at 30fps
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
    
    g_camera.release();
    std::cout << "[Camera] Camera capture stopped" << std::endl;
}

// Video display thread - shows peer's video
void videoDisplayThread() {
    std::cout << "[Display] Starting video display for peer's stream..." << std::endl;
    
    // Initialize decoder
    if (!g_decoder.init()) {
        std::cerr << "[Display] Failed to initialize decoder" << std::endl;
        return;
    }
    
    // Create display window for peer's video
    cv::namedWindow("Peer Video - NovaEngine P2P", cv::WINDOW_NORMAL);
    cv::resizeWindow("Peer Video - NovaEngine P2P", 640, 640);
    cv::setWindowProperty("Peer Video - NovaEngine P2P", cv::WND_PROP_TOPMOST, 1);
    cv::moveWindow("Peer Video - NovaEngine P2P", 100, 100);
    
    std::cout << "[Display] Display window created for peer video" << std::endl;
    
    // Create a default frame to show while waiting
    cv::Mat default_frame = cv::Mat::zeros(640, 640, CV_8UC3);
    cv::putText(default_frame, "Waiting for peer video...", cv::Point(150, 320), 
               cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
    cv::imshow("Peer Video - NovaEngine P2P", default_frame);
    cv::waitKey(1);
    
    std::cout << "[Display] Default frame displayed, waiting for peer video..." << std::endl;
    
    while (g_running) {
        // Check for window close
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q') { // ESC or 'q' to quit
            g_running = false;
            break;
        }
        
        // Small sleep to prevent busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    cv::destroyAllWindows();
    std::cout << "[Display] Video display stopped" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cout << "=== NovaEngine P2P Video Chat ===" << std::endl;
        std::cout << "Usage: " << argv[0] << " <peer_ip> <my_port1> <my_port2> ... | <peer_port1> <peer_port2> ..." << std::endl;
        std::cout << "Example: " << argv[0] << " 192.168.1.100 5001 5002 | 5003 5004" << std::endl;
        std::cout << std::endl;
        std::cout << "P2P Features:" << std::endl;
        std::cout << "  - Send my camera to peer" << std::endl;
        std::cout << "  - Receive peer's camera" << std::endl;
        std::cout << "  - Ultra-low latency (<50ms)" << std::endl;
        std::cout << "  - Multi-tunnel UDP with FEC" << std::endl;
        std::cout << "  - Real-time network adaptation" << std::endl;
        std::cout << "  - H.264 encoding/decoding" << std::endl;
        std::cout << "  - Reed-Solomon error correction" << std::endl;
        return 1;
    }
    
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::string peer_ip = argv[1];
    std::vector<int> my_ports;
    std::vector<int> peer_ports;
    
    bool found_delimiter = false;
    
    // Parse arguments
    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "|") {
            found_delimiter = true;
            continue;
        }
        
        if (!found_delimiter) {
            // My ports (before |)
            my_ports.push_back(std::stoi(arg));
        } else {
            // Peer ports (after |)
            peer_ports.push_back(std::stoi(arg));
        }
    }
    
    if (!found_delimiter) {
        std::cout << "Error: '|' delimiter not found!" << std::endl;
        std::cout << "Usage: " << argv[0] << " <peer_ip> <my_port1> <my_port2> ... | <peer_port1> <peer_port2> ..." << std::endl;
        return 1;
    }
    
    if (my_ports.empty()) {
        std::cout << "Error: No my ports specified!" << std::endl;
        return 1;
    }
    
    if (peer_ports.empty()) {
        std::cout << "Error: No peer ports specified!" << std::endl;
        return 1;
    }
    
    std::cout << "=== NovaEngine P2P Video Chat ===" << std::endl;
    std::cout << "Peer IP: " << peer_ip << std::endl;
    std::cout << "My Ports: ";
    for (size_t i = 0; i < my_ports.size(); i++) {
        std::cout << my_ports[i];
        if (i < my_ports.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
    std::cout << "Peer Ports: ";
    for (size_t i = 0; i < peer_ports.size(); i++) {
        std::cout << peer_ports[i];
        if (i < peer_ports.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
    std::cout << "Initializing P2P components..." << std::endl;
    
    try {
        // Initialize all NovaEngine components
        std::cout << "\n[NovaEngine] Initializing P2P components..." << std::endl;
        
        // Initialize network monitor first
        initNetworkMonitor();
        
        // Encoder: 640x640@30fps, 4Mbps (will be adapted dynamically)
        initEncoder(640, 640, 30, 4000);
        
        // Slicer: 1000 byte chunks, k=6, r=2
        initSlicer(1000, 6, 2);
        
        // Erasure coder: k=6, r=2
        initErasureCoder(6, 2, 1000);
        
        // Scheduler: multiple paths
        initScheduler(peer_ports);
        
        // Collector: 50ms window
        initCollector(50, 6, 2);
        
        // Initialize sender/receiver with my ports and peer IP/ports
        initSenderReceiver(my_ports, peer_ip, peer_ports);
        
        // Set receive callback for P2P communication - decode and display peer's video
        setReceiveCallback([](const std::vector<uint8_t>& frame_data, uint32_t frame_id) {
            std::cout << "[P2P] Received frame " << frame_id << " from peer (" << frame_data.size() << " bytes)" << std::endl;
            
            // Decode and display peer's video
            if (g_display_enabled) {
                cv::Mat decoded_frame;
                if (g_decoder.decode(frame_data, decoded_frame)) {
                    // Check if frame is valid
                    if (!decoded_frame.empty() && decoded_frame.rows > 0 && decoded_frame.cols > 0) {
                        // Add peer indicator text
                        cv::putText(decoded_frame, "NovaEngine P2P - Peer", cv::Point(10, 30), 
                                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
                        cv::putText(decoded_frame, "Frame: " + std::to_string(frame_id), cv::Point(10, 60), 
                                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                        
                        // Display peer's frame
                        cv::imshow("Peer Video - NovaEngine P2P", decoded_frame);
                        int key = cv::waitKey(1);
                        if (key == 27 || key == 'q') {
                            g_running = false;
                        }
                        
                        std::cout << "[P2P] ✅ Displayed peer's frame " << frame_id << " (" 
                                  << decoded_frame.cols << "x" << decoded_frame.rows << ")" << std::endl;
                    } else {
                        std::cerr << "[P2P] ❌ Decoded frame is empty or invalid!" << std::endl;
                    }
                } else {
                    std::cerr << "[P2P] ❌ Failed to decode frame " << frame_id << std::endl;
                }
            }
            
            // Update network monitor with received frame
            NetworkMonitor* monitor = getNetworkMonitor();
            if (monitor) {
                uint64_t receive_timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                monitor->onFrameReceived(frame_id, receive_timestamp);
            }
        });
        
        std::cout << "[NovaEngine] ✅ All P2P components initialized" << std::endl;
        
        // Start all components
        std::cout << "[NovaEngine] Starting P2P components..." << std::endl;
        startSenderReceiver();
        startCollectorFlush([](const std::vector<uint8_t>& frame_data, uint32_t frame_id) {
            std::cout << "[P2P] Frame " << frame_id << " processed by collector" << std::endl;
        });
        
        // Start camera capture thread (sends my video to peer)
        std::thread camera_thread(cameraCaptureThread);
        
        // Start video display thread (shows peer's video)
        std::thread display_thread(videoDisplayThread);
        
        std::cout << "[NovaEngine] P2P video chat started successfully!" << std::endl;
        std::cout << "  - Sending my camera to peer" << std::endl;
        std::cout << "  - Receiving peer's camera" << std::endl;
        std::cout << "Press Ctrl+C or ESC to stop" << std::endl;
        
        // Main loop with network adaptation
        while (g_running) {
            // Dynamic network adaptation
            NetworkMonitor* monitor = getNetworkMonitor();
            if (monitor) {
                auto metrics = monitor->getCurrentMetrics();
                auto optimal_params = monitor->calculateOptimalParams();
                
                // Adapt encoder to network conditions
                if (g_encoder) {
                    adaptEncoderToNetwork(metrics.bandwidth_mbps, metrics.loss_ratio, metrics.rtt_ms);
                }
                
                // Update scheduler with real network metrics
                if (g_scheduler) {
                    std::vector<PathStats> path_stats;
                    for (size_t i = 0; i < peer_ports.size(); i++) {
                        // Distribute bandwidth across paths with slight variations
                        double path_bandwidth = metrics.bandwidth_mbps / peer_ports.size();
                        double path_rtt = metrics.rtt_ms * (1.0 + i * 0.1); // Slight RTT variation
                        double path_loss = metrics.loss_ratio * (1.0 + i * 0.05); // Slight loss variation
                        path_stats.emplace_back(peer_ip, peer_ports[i], path_rtt, path_loss, path_bandwidth);
                    }
                    updateSchedulerMetrics(path_stats);
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Cleanup all components
        std::cout << "[NovaEngine] Shutting down P2P components..." << std::endl;
        
        // Join threads
        if (camera_thread.joinable()) {
            camera_thread.join();
        }
        if (display_thread.joinable()) {
            display_thread.join();
        }
        
        if (g_sender_receiver) {
            stopSenderReceiver();
            shutdownSenderReceiver();
        }
        if (g_collector) shutdownCollector();
        if (g_scheduler) shutdownScheduler();
        if (g_erasure_coder) shutdownErasureCoder();
        if (g_slicer) shutdownSlicer();
        if (g_encoder) shutdownEncoder();
        shutdownNetworkMonitor();
        
    } catch (const std::exception& e) {
        std::cerr << "[NovaEngine] Error: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "[NovaEngine] P2P shutdown complete" << std::endl;
    return 0;
} 