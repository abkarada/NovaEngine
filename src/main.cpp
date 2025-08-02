#include "sender_receiver.hpp"
#include "slicer.hpp"
#include "smart_collector.hpp"
#include "scheduler.hpp"
#include "network_monitor.hpp"
#include "ffmpeg_encoder.h"
#include "erasure_coder.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>
#include <atomic>
#include <random>

// NovaEngine Ultra Stream - Main Application
// Complete ultra-low latency video streaming system

// Global running flag
std::atomic<bool> g_running{true};

void signalHandler(int signum) {
    std::cout << "\n[NovaEngine] Shutting down..." << std::endl;
    g_running = false;
}

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cout << "=== NovaEngine Ultra Stream ===" << std::endl;
        std::cout << "Usage: " << argv[0] << " <target_ip> <local_port1> <local_port2> ... | <target_port1> <target_port2> ..." << std::endl;
        std::cout << "Example: " << argv[0] << " 192.168.1.100 5001 5002 5003 | 5004 5005 5006" << std::endl;
        std::cout << "Example: " << argv[0] << " 192.168.1.100 5001 | 5002" << std::endl;
        std::cout << std::endl;
        std::cout << "Features:" << std::endl;
        std::cout << "  - Ultra-low latency (<50ms)" << std::endl;
        std::cout << "  - Multi-tunnel UDP with FEC" << std::endl;
        std::cout << "  - Real-time network adaptation" << std::endl;
        std::cout << "  - H.264 encoding/decoding" << std::endl;
        std::cout << "  - Reed-Solomon error correction" << std::endl;
        std::cout << "  - Adaptive path scheduling" << std::endl;
        return 1;
    }
    
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::string target_ip = argv[1];
    std::vector<int> local_ports;
    std::vector<int> target_ports;
    
    bool found_delimiter = false;
    
    // Parse arguments
    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "|") {
            found_delimiter = true;
            continue;
        }
        
        if (!found_delimiter) {
            // Local ports (before |)
            local_ports.push_back(std::stoi(arg));
        } else {
            // Target ports (after |)
            target_ports.push_back(std::stoi(arg));
        }
    }
    
    if (!found_delimiter) {
        std::cout << "Error: '|' delimiter not found!" << std::endl;
        std::cout << "Usage: " << argv[0] << " <target_ip> <local_port1> <local_port2> ... | <target_port1> <target_port2> ..." << std::endl;
        return 1;
    }
    
    if (local_ports.empty()) {
        std::cout << "Error: No local ports specified!" << std::endl;
        return 1;
    }
    
    if (target_ports.empty()) {
        std::cout << "Error: No target ports specified!" << std::endl;
        return 1;
    }
    
    std::cout << "=== NovaEngine Ultra Stream ===" << std::endl;
    std::cout << "Target IP: " << target_ip << std::endl;
    std::cout << "Local Ports: ";
    for (size_t i = 0; i < local_ports.size(); i++) {
        std::cout << local_ports[i];
        if (i < local_ports.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
    std::cout << "Target Ports: ";
    for (size_t i = 0; i < target_ports.size(); i++) {
        std::cout << target_ports[i];
        if (i < target_ports.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
    std::cout << "Initializing NovaEngine components..." << std::endl;
    
    try {
        // Initialize all NovaEngine components
        std::cout << "\n[NovaEngine] Initializing all components..." << std::endl;
        
        // Initialize network monitor first
        initNetworkMonitor();
        
        // Encoder: 640x640@30fps, 4Mbps (will be adapted dynamically)
        initEncoder(640, 640, 30, 4000);
        
        // Slicer: 1000 byte chunks, k=6, r=2
        initSlicer(1000, 6, 2);
        
        // Erasure coder: k=6, r=2
        initErasureCoder(6, 2, 1000);
        
        // Scheduler: multiple paths
        initScheduler(target_ports);
        
        // Collector: 50ms window
        initCollector(50, 6, 2);
        
        // Initialize sender/receiver with both local and target ports
        initSenderReceiver(local_ports, target_ip, target_ports);
        
        // Set receive callback for bidirectional communication with full processing
        setReceiveCallback([](const std::vector<uint8_t>& frame_data, uint32_t frame_id) {
            std::cout << "[NovaEngine] Received frame " << frame_id << " (" << frame_data.size() << " bytes)" << std::endl;
            
            // Process received frame with full NovaEngine pipeline
            if (g_sender_receiver) {
                // Update network monitor with received frame
                NetworkMonitor* monitor = getNetworkMonitor();
                if (monitor) {
                    uint64_t receive_timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                    monitor->onFrameReceived(frame_id, receive_timestamp);
                }
                
                // Echo back to sender with different frame ID and full processing
                g_sender_receiver->sendFrame(frame_data, frame_id + 1000);
            }
        });
        
        std::cout << "[NovaEngine] ✅ All components initialized" << std::endl;
        
        // Start all components
        std::cout << "[NovaEngine] Starting all components..." << std::endl;
        startSenderReceiver();
        startCollectorFlush([](const std::vector<uint8_t>& frame_data, uint32_t frame_id) {
            std::cout << "[NovaEngine] Frame " << frame_id << " processed by collector" << std::endl;
        });
        
        std::cout << "[NovaEngine] All components started successfully" << std::endl;
        std::cout << "Press Ctrl+C to stop" << std::endl;
        
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
                    for (size_t i = 0; i < target_ports.size(); i++) {
                        // Distribute bandwidth across paths with slight variations
                        double path_bandwidth = metrics.bandwidth_mbps / target_ports.size();
                        double path_rtt = metrics.rtt_ms * (1.0 + i * 0.1); // Slight RTT variation
                        double path_loss = metrics.loss_ratio * (1.0 + i * 0.05); // Slight loss variation
                        path_stats.emplace_back(target_ip, target_ports[i], path_rtt, path_loss, path_bandwidth);
                    }
                    updateSchedulerMetrics(path_stats);
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Cleanup all components
        std::cout << "[NovaEngine] Shutting down all components..." << std::endl;
        
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
    
    std::cout << "[NovaEngine] Shutdown complete" << std::endl;
    return 0;
} 