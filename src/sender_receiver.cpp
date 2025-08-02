#include "sender_receiver.hpp"
#include "ffmpeg_encoder.h"
#include "slicer.hpp"
#include "erasure_coder.hpp"
#include "udp_sender.hpp"
#include "packet_parser.hpp"
#include "smart_collector.hpp"
#include "decode_and_display.hpp"
#include "rtt_monitor.hpp"
#include "loss_tracker.hpp"

#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <fcntl.h>
#include <iostream>
#include <thread>
#include <vector>
#include <chrono>
#include <deque>
#include <algorithm>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

using namespace std;
using namespace cv;
typedef chrono::steady_clock Clock;

enum StatField { CAPTURE, ENCODE, SEND, DISPLAY, FIELD_COUNT };

// Enhanced bitrate adaptation with network monitoring
class AdaptiveBitrateController {
private:
    static constexpr int WINDOW_SIZE = 10; // 10-second window
    static constexpr double TARGET_LATENCY_MS = 100.0;
    static constexpr double MAX_PACKET_LOSS_RATE = 0.05; // 5%
    
    deque<pair<Clock::time_point, double>> throughput_history;
    deque<pair<Clock::time_point, double>> rtt_history;
    deque<pair<Clock::time_point, double>> loss_history;
    
    int current_bitrate;
    int target_fps;
    
public:
    AdaptiveBitrateController(int initial_bitrate, int fps) 
        : current_bitrate(initial_bitrate), target_fps(fps) {}
    
    void updateMetrics(double throughput_kbps, double rtt_ms, double loss_rate) {
        auto now = Clock::now();
        
        // Update throughput history
        throughput_history.push_back({now, throughput_kbps});
        if (throughput_history.size() > WINDOW_SIZE) {
            throughput_history.pop_front();
        }
        
        // Update RTT history
        rtt_history.push_back({now, rtt_ms});
        if (rtt_history.size() > WINDOW_SIZE) {
            rtt_history.pop_front();
        }
        
        // Update loss history
        loss_history.push_back({now, loss_rate});
        if (loss_history.size() > WINDOW_SIZE) {
            loss_history.pop_front();
        }
    }
    
    int getOptimalBitrate() {
        if (throughput_history.size() < 3) return current_bitrate;
        
        // Calculate moving averages
        double avg_throughput = 0, avg_rtt = 0, avg_loss = 0;
        for (const auto& [_, val] : throughput_history) avg_throughput += val;
        for (const auto& [_, val] : rtt_history) avg_rtt += val;
        for (const auto& [_, val] : loss_history) avg_loss += val;
        
        avg_throughput /= throughput_history.size();
        avg_rtt /= rtt_history.size();
        avg_loss /= loss_history.size();
        
        // Determine optimal bitrate based on network conditions
        int optimal_bitrate = current_bitrate;
        
        // If network is congested (high RTT or loss), reduce bitrate
        if (avg_rtt > TARGET_LATENCY_MS * 1.5 || avg_loss > MAX_PACKET_LOSS_RATE) {
            if (current_bitrate > 1000000) optimal_bitrate = 1000000;      // 1 Mbps
            else if (current_bitrate > 800000) optimal_bitrate = 800000;   // 800 kbps
            else optimal_bitrate = 600000;                                 // 600 kbps
        }
        // If network is good and we have headroom, increase bitrate
        else if (avg_rtt < TARGET_LATENCY_MS * 0.8 && avg_loss < MAX_PACKET_LOSS_RATE * 0.5) {
            if (avg_throughput > current_bitrate * 1.5) {
                if (current_bitrate < 1000000) optimal_bitrate = 1000000;      // 1 Mbps
                else if (current_bitrate < 1800000) optimal_bitrate = 1800000; // 1.8 Mbps
                else if (current_bitrate < 3000000) optimal_bitrate = 3000000; // 3 Mbps
            }
        }
        
        // Ensure we don't exceed available throughput
        optimal_bitrate = min(optimal_bitrate, static_cast<int>(avg_throughput * 0.8));
        
        // Bitrate tiers: 600k, 1M, 1.8M, 3M
        if (optimal_bitrate <= 800000) optimal_bitrate = 600000;
        else if (optimal_bitrate <= 1400000) optimal_bitrate = 1000000;
        else if (optimal_bitrate <= 2400000) optimal_bitrate = 1800000;
        else optimal_bitrate = 3000000;
        
        return optimal_bitrate;
    }
    
    int getOptimalFPS() {
        // Adjust FPS based on bitrate to maintain quality
        if (current_bitrate <= 1000000) return 20;
        else if (current_bitrate <= 1800000) return 25;
        else return 30;
    }
    
    void setCurrentBitrate(int bitrate) { current_bitrate = bitrate; }
    int getCurrentBitrate() const { return current_bitrate; }
};

void run_sender(const string& target_ip, const vector<int>& target_ports) {
    if (target_ports.empty()) {
        cerr << "Usage: sender <target_ip> <port1> [port2 ...]" << endl;
        exit(1);
    }

    if (!init_udp_sockets(target_ports)) {
        cerr << "[ERROR] UDP sockets could not be initialized." << endl;
        exit(1);
    }

    // Initialize target addresses for zero-copy optimization
    set_target_addresses(target_ip, target_ports);

    int width = 640, height = 480, fps = 30, bitrate = 600000;
    VideoCapture cap(0, cv::CAP_V4L2);
    cap.set(CAP_PROP_FRAME_WIDTH, width);
    cap.set(CAP_PROP_FRAME_HEIGHT, height);
    cap.set(CAP_PROP_FPS, fps);
    if (!cap.isOpened()) {
        cerr << "Camera could not be started." << endl;
        exit(1);
    }

    FFmpegEncoder encoder(width, height, fps, bitrate);
    uint16_t frame_id = 0;
    Mat frame;
    ErasureCoder fec(3, 0); // k = 3, r = 0 (no FEC, simple packet collection)

    // Enhanced network monitoring
    RTTMonitor rtt_monitor;
    LossTracker loss_tracker;
    AdaptiveBitrateController bitrate_controller(bitrate, fps);

    // Set up bidirectional receive callback for sender
    set_sender_callback([&](const ChunkPacket& pkt, int socket_id) {
        // Handle incoming ACK packets
        cout << "[sender] Received packet on socket " << socket_id 
             << " (frame " << pkt.frame_id << ", chunk " << (int)pkt.chunk_id << ")" << endl;
        
        // Calculate RTT if this is an ACK packet
        if (pkt.timestamp > 0) {
            auto now_us = chrono::duration_cast<chrono::microseconds>(
                Clock::now().time_since_epoch()).count();
            auto rtt_us = now_us - pkt.timestamp;
            auto rtt_ms = rtt_us / 1000.0;
            
            // Update RTT monitor
            rtt_monitor.receivePong(target_ports[socket_id], pkt.timestamp);
            
            // Log RTT occasionally
            static int rtt_log_counter = 0;
            if (++rtt_log_counter % 50 == 0) {
                cout << "[sender] Socket " << socket_id << " RTT: " << rtt_ms << "ms" << endl;
            }
        }
    });

    // Start bidirectional receive for sender
    start_bidirectional_receive();

    chrono::milliseconds frame_duration(1000 / fps);
    double stats[FIELD_COUNT] = {0};
    int frame_count = 0;
    auto stats_start = Clock::now();

    // Enhanced throughput measurement
    size_t bytes_sent = 0;
    size_t packets_sent = 0;
    auto throughput_start = Clock::now();
    
    // Network metrics collection
    auto metrics_start = Clock::now();

    cout << "[sender] Starting bidirectional sender loop..." << endl;

    while (true) {
        auto t0 = Clock::now();

        auto tc0 = Clock::now();
        cap.read(frame);
        auto tc1 = Clock::now();
        stats[CAPTURE] += chrono::duration<double, milli>(tc1 - tc0).count();

        if (!frame.empty()) {
            auto te0 = Clock::now();
            vector<uint8_t> encoded;
            encoder.encodeFrame(frame, encoded);
            auto te1 = Clock::now();
            stats[ENCODE] += chrono::duration<double, milli>(te1 - te0).count();

            cout << "[sender] Encoded frame " << frame_id << " to " << encoded.size() << " bytes" << endl;

            auto ts0 = Clock::now();
            auto chunks = slice_frame(encoded, frame_id, 1000); // chunk_size = 1000
            cout << "[sender] Sliced into " << chunks.size() << " chunks" << endl;

            vector<vector<uint8_t>> k_blocks;
            for (int i = 0; i < 3 && i < chunks.size(); ++i) {
                k_blocks.push_back(chunks[i].payload);
                cout << "[sender] Chunk " << i << " size: " << chunks[i].payload.size() << endl;
            }
            // If less than k, pad with zeros (already chunk_size in slicing)
            while (k_blocks.size() < 3) {
                k_blocks.push_back(vector<uint8_t>(1000, 0));
                cout << "[sender] Padded chunk to 1000 bytes (total " << k_blocks.size() << ")" << endl;
            }

            vector<vector<uint8_t>> all_blocks;
            fec.encode(k_blocks, all_blocks);
            cout << "[sender] FEC encoded to " << all_blocks.size() << " blocks" << endl;

            // Enhanced multipath sending with weighted scheduling
            for (int i = 0; i < all_blocks.size(); ++i) {
                ChunkPacket pkt;
                pkt.frame_id = frame_id;
                pkt.chunk_id = i;
                pkt.total_chunks = all_blocks.size();
                pkt.payload = all_blocks[i];
                pkt.timestamp = chrono::duration_cast<chrono::microseconds>(
                    Clock::now().time_since_epoch()).count();

                // Track packet sending for loss calculation
                for (int p : target_ports) {
                    loss_tracker.packetSent(p);
                }

                // Use enhanced multipath sending: send each chunk on a different port (round robin)
                int port_idx = i % target_ports.size();
                ssize_t sent = send_udp(target_ip, target_ports[port_idx], pkt);
                if (sent > 0) {
                    bytes_sent += sent;
                    packets_sent++;
                    // Track RTT for this path (using first port as representative)
                    if (i == 0) rtt_monitor.startPing(target_ports[0], pkt.timestamp);
                }
            }

            // Enhanced bitrate adaptation every second
            auto now_tp = Clock::now();
            if (chrono::duration_cast<chrono::seconds>(now_tp - throughput_start).count() >= 1) {
                double throughput_kbps = (bytes_sent * 8) / 1000.0;
                double avg_rtt = rtt_monitor.getAverageRTT();
                double loss_rate = loss_tracker.getLossRate();
                
                // Update bitrate controller
                bitrate_controller.updateMetrics(throughput_kbps, avg_rtt, loss_rate);
                int optimal_bitrate = bitrate_controller.getOptimalBitrate();
                int optimal_fps = bitrate_controller.getOptimalFPS();
                
                // Apply bitrate change if needed
                if (optimal_bitrate != encoder.getBitrate()) {
                    cout << "[ADAPTIVE] Bitrate: " << encoder.getBitrate()/1000 
                         << "k -> " << optimal_bitrate/1000 << "k, RTT: " 
                         << avg_rtt << "ms, Loss: " << (loss_rate*100) << "%" << endl;
                    encoder.setBitrate(optimal_bitrate);
                    bitrate_controller.setCurrentBitrate(optimal_bitrate);
                }
                
                // Apply FPS change if needed
                if (optimal_fps != fps) {
                    cout << "[ADAPTIVE] FPS: " << fps << " -> " << optimal_fps << endl;
                    fps = optimal_fps;
                    frame_duration = chrono::milliseconds(1000 / fps);
                }
                
                bytes_sent = 0;
                packets_sent = 0;
                throughput_start = now_tp;
            }

            auto ts1 = Clock::now();
            stats[SEND] += chrono::duration<double, milli>(ts1 - ts0).count();
            frame_id++;
        }

        auto td0 = Clock::now();
        imshow("NovaEngine - Sender (You)", frame);
        if (waitKey(1) >= 0) break;
        auto td1 = Clock::now();
        stats[DISPLAY] += chrono::duration<double, milli>(td1 - td0).count();

        // Display received frame if in both mode
        // if (has_received && !reconstructed_frame.empty()) { // This line is removed as per the edit hint
        //     Mat display_frame;
        //     Mat resized;
        //     resize(reconstructed_frame, resized, Size(640, 480));
        //     display_frame = resized;
            
        //     imshow("NovaEngine - Both Mode", display_frame);
        //     if (waitKey(1) == 27) break;
        // }

        // Flush expired frames
        // collector.flush_expired_frames(); // This line is removed as per the edit hint

        frame_count++;
        auto now = Clock::now();
        if (chrono::duration_cast<chrono::seconds>(now - stats_start).count() >= 1) {
            cout << "Avg times (ms): Capture=" << stats[CAPTURE]/frame_count
                 << ", Encode=" << stats[ENCODE]/frame_count
                 << ", Send=" << stats[SEND]/frame_count
                 << ", Display=" << stats[DISPLAY]/frame_count 
                 << ", RTT=" << rtt_monitor.getAverageRTT() << "ms"
                 << ", Loss=" << (loss_tracker.getLossRate()*100) << "%" << endl;
            memset(stats, 0, sizeof(stats));
            frame_count = 0;
            stats_start = now;
        }

        auto t1 = Clock::now();
        auto loop_ms = chrono::duration_cast<chrono::milliseconds>(t1 - t0);
        if (loop_ms < frame_duration) {
            this_thread::sleep_for(frame_duration - loop_ms);
        }
    }

    // Stop bidirectional receive thread
    stop_bidirectional_receive();
    
    close_udp_sockets();
    destroyAllWindows();
}

void run_receiver(const vector<int>& ports) {
    constexpr int MAX_BUFFER = 1500;
    const int disp_width = 640;
    const int disp_height = 480;

    // Use global sockets instead of creating new ones
    // Don't reinitialize if already done
    vector<int> sockets = get_global_sockets();
    if (sockets.empty()) {
        // Only initialize if not already done
        if (!init_global_sockets(ports)) {
            cerr << "[ERROR] Failed to initialize global sockets!" << endl;
            return;
        }
        sockets = get_global_sockets();
    }
    
    if (sockets.empty()) {
        cerr << "[ERROR] No sockets available!" << endl;
        return;
    }
    
    cout << "[receiver] Using " << sockets.size() << " global sockets for bidirectional I/O..." << endl;

    H264Decoder decoder;
    Mat reconstructed_frame;
    bool has_received = false;

    SmartFrameCollector collector([&](const vector<uint8_t>& data) {
        if (decoder.decode(data, reconstructed_frame)) {
            has_received = true;
        }
    }, 3, 0); // k = 3, r = 0 (no FEC, simple packet collection)

    // Set up bidirectional receive callback
    set_receiver_callback([&](const ChunkPacket& pkt, int socket_id) {
        // Handle incoming video packets
        cout << "[receiver] Received packet on socket " << socket_id 
             << " (frame " << pkt.frame_id << ", chunk " << (int)pkt.chunk_id 
             << "/" << (int)pkt.total_chunks << ", size=" << pkt.payload.size() << ")" << endl;
        
        // Check if this is an ACK packet (small size and total_chunks=1)
        if (pkt.total_chunks == 1 && pkt.payload.size() <= 10) {
            cout << "[receiver] Ignoring ACK packet (frame=" << pkt.frame_id 
                 << ", chunk=" << (int)pkt.chunk_id << ")" << endl;
            return; // Don't process ACK packets
        }
        
        // This is a video packet, send ACK back to sender
        ChunkPacket ack_pkt;
        ack_pkt.frame_id = pkt.frame_id;
        ack_pkt.chunk_id = pkt.chunk_id;
        ack_pkt.total_chunks = 1; // ACK packet
        ack_pkt.timestamp = chrono::duration_cast<chrono::microseconds>(
            Clock::now().time_since_epoch()).count();
        ack_pkt.payload = {0x41, 0x43, 0x4B}; // "ACK"
        
        // Send ACK back to sender
        send_udp("127.0.0.1", ports[socket_id], ack_pkt);
        
        // Process the video packet
        cout << "[receiver] Processing VIDEO packet: frame=" << pkt.frame_id 
             << ", chunk=" << (int)pkt.chunk_id << ", size=" << pkt.payload.size() << endl;
        collector.handle(pkt);
    });

    // Start bidirectional receive thread
    cout << "[receiver] Starting bidirectional receive..." << endl;
    start_bidirectional_receive();

    cout << "Receiver started with " << sockets.size() << " sockets in bidirectional mode..." << endl;

    while (true) {
        // Main loop now only handles display and frame processing
        // All packet receiving is handled by the bidirectional thread
        
        collector.flush_expired_frames();

        Mat display_frame;
        if (has_received && !reconstructed_frame.empty()) {
            Mat resized;
            resize(reconstructed_frame, resized, Size(disp_width, disp_height));
            display_frame = resized;
        } else {
            display_frame = Mat::zeros(disp_height, disp_width, CV_8UC3);
            putText(display_frame,
                    "Waiting for Client to Connect..",
                    Point(20, disp_height/2),
                    FONT_HERSHEY_SIMPLEX,
                    1.0,
                    Scalar(255,255,255),
                    2);
        }

        imshow("NovaEngine - Receiver", display_frame);
        if (waitKey(1) == 27) break;
        this_thread::sleep_for(chrono::milliseconds(1));
    }

    // Stop bidirectional receive thread
    stop_bidirectional_receive();
    
    // Don't close sockets here, they're managed globally
    destroyAllWindows();
}
