#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <signal.h>
#include <ctime>

// NovaEngine Ultra Stream headers
#include "ffmpeg_encoder.h"
#include "slicer.hpp"
#include "erasure_coder.hpp"
#include "udp_sender.hpp"
#include "scheduler.hpp"
#include "smart_collector.hpp"
#include "sender_receiver.hpp"
#include "decode_and_display.hpp"

using namespace cv;
using namespace std;

// Global state
static atomic<bool> running{true};
static atomic<bool> shutdown_requested{false};

// Signal handler for graceful shutdown
void signalHandler(int signum) {
    cout << "\n[Main] Received signal " << signum << ", shutting down gracefully..." << endl;
    shutdown_requested = true;
    running = false;
}

// Frame display callback
void onFrameReceived(const vector<uint8_t>& frame_data, uint32_t frame_id) {
    static H264Decoder decoder;
    static Mat display_frame;
    
    // Decode frame
    if (decoder.decode(frame_data, display_frame)) {
        // Display frame
        imshow("NovaEngine - Received Frame", display_frame);
        waitKey(1);
        
        cout << "[Main] Displayed frame " << frame_id << " (" << frame_data.size() << " bytes)" << endl;
    }
}

// Network adaptation thread
void networkAdaptationThread() {
    cout << "[Main] Network adaptation thread started" << endl;
    
    while (running) {
        // Simulate network monitoring (in real implementation, this would query actual network metrics)
        double network_kbps = 3000.0;  // Simulated network capacity
        double loss_ratio = 0.02;      // Simulated 2% loss
        
        // Adapt encoder to network conditions
        if (g_encoder) {
            adaptEncoderToNetwork(network_kbps, loss_ratio);
        }
        
        // Update scheduler metrics
        if (g_scheduler) {
            vector<PathStats> path_stats;
            path_stats.emplace_back("127.0.0.1", 5001, 25.0, 0.01);
            path_stats.emplace_back("127.0.0.1", 5002, 30.0, 0.02);
            path_stats.emplace_back("127.0.0.1", 5003, 35.0, 0.03);
            
            updateSchedulerMetrics(path_stats);
        }
        
        // Sleep for adaptation interval
        this_thread::sleep_for(chrono::milliseconds(100));
    }
    
    cout << "[Main] Network adaptation thread stopped" << endl;
}

// Statistics monitoring thread
void statisticsThread() {
    cout << "[Main] Statistics monitoring thread started" << endl;
    
    while (running) {
        // Get statistics from all components
        if (g_encoder && g_scheduler && g_collector && g_sender_receiver) {
            auto encoder_stats = g_encoder->getStats();
            auto scheduler_stats = g_scheduler->getStats();
            auto collector_stats = g_collector->getStats();
            auto sender_stats = g_sender_receiver->getStats();
            
            // Print statistics every 5 seconds
            static int counter = 0;
            if (++counter % 50 == 0) {  // 50 * 100ms = 5 seconds
                cout << "\n=== NovaEngine Statistics ===" << endl;
                cout << "Encoder: " << encoder_stats.frames_encoded << " frames, "
                     << encoder_stats.avg_bitrate_kbps << " kbps, "
                     << encoder_stats.current_fps << " fps" << endl;
                cout << "Scheduler: " << scheduler_stats.total_chunks_sent << " chunks, "
                     << scheduler_stats.avg_rtt_ms << "ms RTT, "
                     << (scheduler_stats.avg_loss_ratio * 100) << "% loss" << endl;
                cout << "Collector: " << collector_stats.frames_completed << "/" 
                     << collector_stats.frames_received << " frames, "
                     << collector_stats.avg_latency_ms << "ms latency" << endl;
                cout << "Sender: " << sender_stats.frames_sent << " sent, "
                     << sender_stats.frames_received << " received, "
                     << sender_stats.throughput_mbps << " Mbps" << endl;
                cout << "=============================" << endl;
            }
        }
        
        this_thread::sleep_for(chrono::milliseconds(100));
    }
    
    cout << "[Main] Statistics monitoring thread stopped" << endl;
}

int main(int argc, char* argv[]) {
    if (argc < 4) {
        cout << "Usage: " << argv[0] << " <mode> <target_ip> <target_port> [local_port]" << endl;
        cout << "  mode: sender, receiver, or both" << endl;
        cout << "  target_ip: IP address of target" << endl;
        cout << "  target_port: Port of target" << endl;
        cout << "  local_port: Local port (optional, default: target_port+1)" << endl;
        return 1;
    }
    
    string mode = argv[1];
    string target_ip = argv[2];
    int target_port = stoi(argv[3]);
    int local_port = (argc > 4) ? stoi(argv[4]) : target_port + 1;
    
    // Set up signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    cout << "=== NovaEngine Ultra Stream v1.0 ===" << endl;
    cout << "Mode: " << mode << endl;
    cout << "Target: " << target_ip << ":" << target_port << endl;
    cout << "Local port: " << local_port << endl;
    cout << "=====================================" << endl;
    
    try {
        // Initialize all NovaEngine components
        cout << "\n[Main] Initializing NovaEngine components..." << endl;
        
        // Initialize encoder (1280x720@30fps, 3Mbps)
        initEncoder(1280, 720, 30, 3000);
        
        // Initialize slicer (1000 byte chunks, k=6, r=2)
        initSlicer(1000, 6, 2);
        
        // Initialize erasure coder (k=6, r=2, 1000 byte chunks)
        initErasureCoder(6, 2, 1000);
        
        // Initialize scheduler with multiple paths
        vector<int> ports = {5001, 5002, 5003};
        initScheduler(ports);
        
        // Initialize collector (50ms window, k=6, r=2)
        initCollector(50, 6, 2);
        
        // Initialize sender/receiver
        vector<int> local_ports = {local_port, local_port + 1, local_port + 2};
        vector<int> target_ports = {target_port, target_port + 1, target_port + 2};
        initSenderReceiver(local_ports, target_ip, target_ports);
        
        cout << "[Main] ✅ All components initialized successfully" << endl;
        
        // Set up frame receive callback
        setReceiveCallback(onFrameReceived);
        
        // Start collector flush thread
        startCollectorFlush(onFrameReceived);
        
        // Start sender/receiver
        startSenderReceiver();
        
        // Start background threads
        thread network_thread(networkAdaptationThread);
        thread stats_thread(statisticsThread);
        
        if (mode == "sender" || mode == "both") {
            cout << "\n[Main] Starting sender mode..." << endl;
            
            // Force test pattern mode for now (camera access issues)
            cout << "[Main] Using test pattern mode (camera access disabled)" << endl;
            
            Mat frame;
            uint32_t frame_id = 0;
            
            cout << "[Main] Test pattern loop started. Press 'q' to quit." << endl;
            
            while (running && !shutdown_requested) {
                // Create a test pattern frame
                frame = Mat::zeros(720, 1280, CV_8UC3);
                
                // Draw some test content
                putText(frame, "NovaEngine Test Pattern", Point(50, 100), 
                       FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
                putText(frame, "Frame: " + to_string(frame_id), Point(50, 200), 
                       FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2);
                putText(frame, "Time: " + to_string(time(nullptr)), Point(50, 300), 
                       FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2);
                
                // Draw a moving circle
                int x = 200 + 100 * sin(frame_id * 0.1);
                int y = 400 + 100 * cos(frame_id * 0.1);
                circle(frame, Point(x, y), 50, Scalar(0, 0, 255), -1);
                
                // Draw network stats
                putText(frame, "Network: 3000 kbps", Point(50, 500), 
                       FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 0), 2);
                putText(frame, "FEC: k=6, r=2", Point(50, 550), 
                       FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 0), 2);
                putText(frame, "Multi-tunnel: 3 UDP ports", Point(50, 600), 
                       FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 0), 2);
                
                // Encode frame
                vector<uint8_t> encoded_data;
                if (encodeFrame(frame, encoded_data)) {
                    // Send frame
                    if (sendFrame(encoded_data, frame_id)) {
                        cout << "[Main] Sent test frame " << frame_id << " (" << encoded_data.size() << " bytes)" << endl;
                    }
                    frame_id++;
                }
                
                // Display local frame
                imshow("NovaEngine - Test Pattern", frame);
                
                // Check for quit
                char key = waitKey(1);
                if (key == 'q' || key == 27) {
                    shutdown_requested = true;
                    break;
                }
                
                // Frame rate control
                this_thread::sleep_for(chrono::milliseconds(33));  // ~30 fps
            }
        }
        
        if (mode == "receiver" || mode == "both") {
            cout << "\n[Main] Starting receiver mode..." << endl;
            
            // Create a window for received frames
            namedWindow("NovaEngine - Received Frame", WINDOW_AUTOSIZE);
            
            cout << "[Main] Receiver loop started. Press 'q' to quit." << endl;
            
            while (running && !shutdown_requested) {
                // Check for quit
                char key = waitKey(1);
                if (key == 'q' || key == 27) {
                    shutdown_requested = true;
                    break;
                }
                
                this_thread::sleep_for(chrono::milliseconds(10));
            }
        }
        
        // Wait for background threads
        if (network_thread.joinable()) {
            network_thread.join();
        }
        if (stats_thread.joinable()) {
            stats_thread.join();
        }
        
    } catch (const exception& e) {
        cerr << "[Main] Error: " << e.what() << endl;
        return 1;
    }
    
    // Cleanup
    cout << "\n[Main] Shutting down NovaEngine..." << endl;
    
    if (g_sender_receiver) {
        stopSenderReceiver();
        shutdownSenderReceiver();
    }
    if (g_collector) {
        shutdownCollector();
    }
    if (g_scheduler) {
        shutdownScheduler();
    }
    if (g_erasure_coder) {
        shutdownErasureCoder();
    }
    if (g_slicer) {
        shutdownSlicer();
    }
    if (g_encoder) {
        shutdownEncoder();
    }
    
    destroyAllWindows();
    
    cout << "[Main] NovaEngine shutdown complete" << endl;
    return 0;
}

