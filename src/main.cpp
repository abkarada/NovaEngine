#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include "ffmpeg_encoder.h"
#include "decode_and_display.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <map>
#include <vector>
#include <chrono>
#include <iostream>
#include <cstring>
#include <mutex>
#include <jerasure.h>
#include <reed_sol.h>

using namespace cv;
using namespace std;

// Global state
static atomic<bool> running{true};
static int udp_socket = -1;
static constexpr int CHUNK_SIZE = 1000;
static constexpr int FRAME_TIMEOUT_MS = 50;
static constexpr int K_CHUNKS = 6;  // Data chunks
static constexpr int R_CHUNKS = 2;  // Parity chunks
static constexpr int TOTAL_CHUNKS = K_CHUNKS + R_CHUNKS;

// Reed-Solomon FEC setup
static int* rs_matrix = nullptr;
static int* rs_bitmatrix = nullptr;

// Thread-safe frame buffer with FEC support
struct FrameBuffer {
    map<uint16_t, vector<vector<uint8_t>>> chunks;
    map<uint16_t, vector<bool>> chunk_received;
    map<uint16_t, chrono::steady_clock::time_point> timestamps;
    mutex mtx;
    
    void add_chunk(uint16_t frame_id, uint8_t chunk_id, vector<uint8_t>&& payload) {
        lock_guard<mutex> lock(mtx);
        
        if (chunks[frame_id].empty()) {
            chunks[frame_id].resize(TOTAL_CHUNKS);
            chunk_received[frame_id].resize(TOTAL_CHUNKS, false);
            timestamps[frame_id] = chrono::steady_clock::now();
        }
        
        if (chunk_id < TOTAL_CHUNKS) {
            chunks[frame_id][chunk_id] = move(payload);
            chunk_received[frame_id][chunk_id] = true;
        }
    }
    
    bool can_decode_frame(uint16_t frame_id) {
        lock_guard<mutex> lock(mtx);
        auto it = chunk_received.find(frame_id);
        if (it == chunk_received.end()) return false;
        
        int received_count = 0;
        for (bool received : it->second) {
            if (received) received_count++;
        }
        
        return received_count >= K_CHUNKS;  // Need at least k chunks to decode
    }
    
    vector<uint8_t> decode_frame(uint16_t frame_id) {
        lock_guard<mutex> lock(mtx);
        auto it = chunks.find(frame_id);
        auto received_it = chunk_received.find(frame_id);
        
        if (it == chunks.end() || received_it == chunk_received.end()) {
            return {};
        }
        
        // Count received chunks and create erasures list
        int received_count = 0;
        vector<int> received_chunks;
        vector<int> erasures;
        
        for (int i = 0; i < TOTAL_CHUNKS; ++i) {
            if (received_it->second[i]) {
                received_count++;
                received_chunks.push_back(i);
            } else {
                erasures.push_back(i);
            }
        }
        
        if (received_count < K_CHUNKS) {
            return {};  // Not enough chunks
        }
        
        // Prepare data for FEC decode
        char** data_ptrs = new char*[TOTAL_CHUNKS];
        char** coding_ptrs = new char*[R_CHUNKS];
        
        for (int i = 0; i < TOTAL_CHUNKS; ++i) {
            data_ptrs[i] = new char[CHUNK_SIZE];
            if (received_it->second[i]) {
                memcpy(data_ptrs[i], it->second[i].data(), CHUNK_SIZE);
            } else {
                memset(data_ptrs[i], 0, CHUNK_SIZE);
            }
        }
        
        for (int i = 0; i < R_CHUNKS; ++i) {
            coding_ptrs[i] = data_ptrs[K_CHUNKS + i];
        }
        
        // Perform FEC decode
        int decode_result = jerasure_matrix_decode(K_CHUNKS, R_CHUNKS, 8, 
                                                 rs_matrix, 1, erasures.data(), 
                                                 data_ptrs, coding_ptrs, CHUNK_SIZE);
        
        vector<uint8_t> frame_data;
        if (decode_result == 0) {
            // Successfully decoded, reconstruct frame
            for (int i = 0; i < K_CHUNKS; ++i) {
                frame_data.insert(frame_data.end(), 
                                reinterpret_cast<uint8_t*>(data_ptrs[i]),
                                reinterpret_cast<uint8_t*>(data_ptrs[i]) + CHUNK_SIZE);
            }
        }
        
        // Cleanup
        for (int i = 0; i < TOTAL_CHUNKS; ++i) {
            delete[] data_ptrs[i];
        }
        delete[] data_ptrs;
        delete[] coding_ptrs;
        
        // Remove frame from buffer
        chunks.erase(frame_id);
        chunk_received.erase(frame_id);
        timestamps.erase(frame_id);
        
        return frame_data;
    }
    
    void cleanup_expired() {
        lock_guard<mutex> lock(mtx);
        auto now = chrono::steady_clock::now();
        
        for (auto it = timestamps.begin(); it != timestamps.end();) {
            auto elapsed = chrono::duration_cast<chrono::milliseconds>(now - it->second).count();
            if (elapsed > FRAME_TIMEOUT_MS) {
                chunks.erase(it->first);
                chunk_received.erase(it->first);
                it = timestamps.erase(it);
            } else {
                ++it;
            }
        }
    }
};

static FrameBuffer frame_buffer;

// UDP Socket setup
int setup_udp_socket(int port) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket creation failed");
        return -1;
    }
    
    // Enable address reuse
    int optval = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    
    // Set non-blocking
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    
    // Bind to port
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
    
    if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("bind failed");
        close(sock);
        return -1;
    }
    
    return sock;
}

// Packet structure
struct VideoPacket {
    uint16_t frame_id;
    uint8_t chunk_id;
    uint8_t total_chunks;
    uint8_t data[CHUNK_SIZE];
};

// Sender thread
void udp_send_thread(const string& target_ip, int target_port, int local_port) {
    cout << "[SENDER] Starting sender thread on port " << local_port << endl;
    
    // Setup camera
    VideoCapture cap(0, CAP_V4L2);
    if (!cap.isOpened()) {
        cerr << "[SENDER] Failed to open camera" << endl;
        return;
    }
    
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(CAP_PROP_FPS, 30);
    
    // Setup encoder
    FFmpegEncoder encoder(640, 480, 30, 600000);
    
    // Setup target address
    sockaddr_in target_addr{};
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(target_port);
    inet_pton(AF_INET, target_ip.c_str(), &target_addr.sin_addr);
    
    uint16_t frame_id = 0;
    Mat frame;
    
    while (running) {
        cap.read(frame);
        if (frame.empty()) continue;
        
        // Encode frame
        vector<uint8_t> encoded_data;
        if (!encoder.encodeFrame(frame, encoded_data)) continue;
        
        // Pad encoded data to fit exactly in K_CHUNKS * CHUNK_SIZE
        size_t total_size = K_CHUNKS * CHUNK_SIZE;
        if (encoded_data.size() < total_size) {
            encoded_data.resize(total_size, 0);
        }
        
        // Split into K data chunks
        vector<vector<uint8_t>> data_chunks(K_CHUNKS);
        for (int i = 0; i < K_CHUNKS; ++i) {
            data_chunks[i].resize(CHUNK_SIZE);
            memcpy(data_chunks[i].data(), 
                   encoded_data.data() + i * CHUNK_SIZE, 
                   CHUNK_SIZE);
        }
        
        // Generate R parity chunks using Reed-Solomon FEC
        char** data_ptrs = new char*[K_CHUNKS];
        char** coding_ptrs = new char*[R_CHUNKS];
        
        for (int i = 0; i < K_CHUNKS; ++i) {
            data_ptrs[i] = reinterpret_cast<char*>(data_chunks[i].data());
        }
        
        for (int i = 0; i < R_CHUNKS; ++i) {
            coding_ptrs[i] = new char[CHUNK_SIZE];
        }
        
        // Perform FEC encoding
        jerasure_matrix_encode(K_CHUNKS, R_CHUNKS, 8, rs_matrix, 
                              data_ptrs, coding_ptrs, CHUNK_SIZE);
        
        // Send all K+R chunks
        for (int i = 0; i < TOTAL_CHUNKS; ++i) {
            VideoPacket pkt{};
            pkt.frame_id = frame_id;
            pkt.chunk_id = i;
            pkt.total_chunks = TOTAL_CHUNKS;
            
            if (i < K_CHUNKS) {
                // Data chunk
                memcpy(pkt.data, data_ptrs[i], CHUNK_SIZE);
            } else {
                // Parity chunk
                memcpy(pkt.data, coding_ptrs[i - K_CHUNKS], CHUNK_SIZE);
            }
            
            // Send chunk
            ssize_t sent = sendto(udp_socket, &pkt, sizeof(VideoPacket), 0,
                                reinterpret_cast<sockaddr*>(&target_addr), sizeof(target_addr));
            
            if (sent < 0) {
                cerr << "[SENDER] Send failed: " << strerror(errno) << endl;
            }
        }
        
        // Cleanup
        for (int i = 0; i < R_CHUNKS; ++i) {
            delete[] coding_ptrs[i];
        }
        delete[] data_ptrs;
        delete[] coding_ptrs;
        
        frame_id++;
        this_thread::sleep_for(chrono::milliseconds(33)); // ~30 FPS
    }
    
    cout << "[SENDER] Sender thread stopped" << endl;
}

// Receiver thread
void udp_receive_thread() {
    cout << "[RECEIVER] Starting receiver thread" << endl;
    
    H264Decoder decoder;
    Mat decoded_frame;
    uint8_t recv_buffer[sizeof(VideoPacket)];
    
    while (running) {
        // Cleanup expired frames
        frame_buffer.cleanup_expired();
        
        // Receive packet
        sockaddr_in sender_addr{};
        socklen_t addr_len = sizeof(sender_addr);
        
        ssize_t received = recvfrom(udp_socket, recv_buffer, sizeof(recv_buffer), 0,
                                  reinterpret_cast<sockaddr*>(&sender_addr), &addr_len);
        
        if (received == sizeof(VideoPacket)) {
            VideoPacket* pkt = reinterpret_cast<VideoPacket*>(recv_buffer);
            
            // Extract chunk data
            vector<uint8_t> chunk_data(pkt->data, pkt->data + CHUNK_SIZE);
            
            // Add to frame buffer
            frame_buffer.add_chunk(pkt->frame_id, pkt->chunk_id, move(chunk_data));
            
            // Check if frame is complete
            if (frame_buffer.can_decode_frame(pkt->frame_id)) {
                auto frame_data = frame_buffer.decode_frame(pkt->frame_id);
                
                // Decode frame
                if (decoder.decode(frame_data, decoded_frame)) {
                    // Display frame
                    imshow("NovaEngine - Receiver", decoded_frame);
                    if (waitKey(1) == 27) { // ESC key
                        running = false;
                        break;
                    }
                }
            }
        } else if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            cerr << "[RECEIVER] Recv error: " << strerror(errno) << endl;
        }
        
        this_thread::sleep_for(chrono::microseconds(100)); // Small delay
    }
    
    cout << "[RECEIVER] Receiver thread stopped" << endl;
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        cerr << "Usage: " << argv[0] << " <local_port> <target_ip> <target_port>" << endl;
        return 1;
    }
    
    int local_port = stoi(argv[1]);
    string target_ip = argv[2];
    int target_port = stoi(argv[3]);
    
    // Initialize Reed-Solomon FEC matrix
    rs_matrix = reed_sol_vandermonde_coding_matrix(K_CHUNKS, R_CHUNKS, 8);
    if (!rs_matrix) {
        cerr << "Failed to create Reed-Solomon matrix" << endl;
        return 1;
    }
    
    cout << "[FEC] Initialized Reed-Solomon FEC (k=" << K_CHUNKS << ", r=" << R_CHUNKS << ")" << endl;
    
    // Setup UDP socket
    udp_socket = setup_udp_socket(local_port);
    if (udp_socket < 0) {
        cerr << "Failed to setup UDP socket" << endl;
        free(rs_matrix);
        return 1;
    }
    
    cout << "NovaEngine starting on port " << local_port 
         << ", sending to " << target_ip << ":" << target_port << endl;
    
    // Start threads
    thread sender_thread(udp_send_thread, target_ip, target_port, local_port);
    thread receiver_thread(udp_receive_thread);
    
    // Wait for threads
    sender_thread.join();
    receiver_thread.join();
    
    // Cleanup
    if (udp_socket >= 0) close(udp_socket);
    if (rs_matrix) free(rs_matrix);
    destroyAllWindows();
    
    cout << "NovaEngine stopped" << endl;
    return 0;
}
