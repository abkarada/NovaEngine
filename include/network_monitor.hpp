#pragma once

#include <vector>
#include <map>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <deque>
#include <functional>

// NovaEngine Ultra Stream - Dynamic Network Monitoring
// Real-time packet loss, RTT, bandwidth and frame latency calculation

struct NetworkMetrics {
    double rtt_ms;              // Round-trip time in milliseconds
    double loss_ratio;          // Packet loss ratio (0.0-1.0)
    double bandwidth_mbps;      // Available bandwidth in Mbps
    double jitter_ms;           // Jitter measurement
    double frame_latency_ms;    // Total frame processing latency
    uint64_t packets_sent;      // Total packets sent
    uint64_t packets_received;  // Total packets received
    uint64_t packets_lost;      // Total packets lost
    uint64_t bytes_sent;        // Total bytes sent
    uint64_t bytes_received;    // Total bytes received
    std::chrono::steady_clock::time_point last_update;
    
    NetworkMetrics() : rtt_ms(50.0), loss_ratio(0.0), bandwidth_mbps(10.0), 
                      jitter_ms(5.0), frame_latency_ms(33.0), packets_sent(0), 
                      packets_received(0), packets_lost(0), bytes_sent(0), bytes_received(0) {
        last_update = std::chrono::steady_clock::now();
    }
};

struct FrameMetrics {
    uint32_t frame_id;
    uint64_t send_timestamp;
    uint64_t receive_timestamp;
    uint16_t chunks_sent;
    uint16_t chunks_received;
    uint16_t chunks_lost;
    double frame_size_bytes;
    double motion_entropy;
    bool is_keyframe;
    
    FrameMetrics() : frame_id(0), send_timestamp(0), receive_timestamp(0),
                    chunks_sent(0), chunks_received(0), chunks_lost(0),
                    frame_size_bytes(0.0), motion_entropy(0.0), is_keyframe(false) {}
};

class NetworkMonitor {
public:
    NetworkMonitor();
    ~NetworkMonitor();
    
    // Start monitoring
    void startMonitoring();
    void stopMonitoring();
    
    // Update packet statistics
    void onPacketSent(uint32_t frame_id, uint16_t chunk_id, uint64_t timestamp, size_t data_size);
    void onPacketReceived(uint32_t frame_id, uint16_t chunk_id, uint64_t timestamp, size_t data_size);
    void onPacketLost(uint32_t frame_id, uint16_t chunk_id);
    
    // Update frame statistics
    void onFrameSent(uint32_t frame_id, double frame_size, double motion_entropy, bool is_keyframe);
    void onFrameReceived(uint32_t frame_id, uint64_t receive_timestamp);
    
    // Get current network metrics
    NetworkMetrics getCurrentMetrics() const;
    
    // Calculate optimal parameters
    struct OptimalParams {
        int optimal_bitrate_kbps;
        int optimal_fps;
        int optimal_resolution_width;
        int optimal_resolution_height;
        double optimal_compression_ratio;
        double max_bandwidth_usage;
    };
    
    OptimalParams calculateOptimalParams() const;
    
    // Set callback for metrics updates
    void setMetricsCallback(std::function<void(const NetworkMetrics&)> callback);
    
    // Get frame latency statistics
    double getAverageFrameLatency() const;
    double getFrameLatencyPercentile(double percentile) const;
    
    // Calculate motion entropy (scene complexity)
    double calculateMotionEntropy(const std::vector<uint8_t>& frame_data) const;
    
    // Calculate compression ratio
    double calculateCompressionRatio(double original_size, double compressed_size) const;

private:
    mutable std::mutex metrics_mutex_;
    std::atomic<bool> running_;
    std::thread monitor_thread_;
    
    NetworkMetrics current_metrics_;
    std::map<uint32_t, FrameMetrics> frame_metrics_;
    std::deque<double> rtt_history_;
    std::deque<double> loss_history_;
    std::deque<double> latency_history_;
    
    std::function<void(const NetworkMetrics&)> metrics_callback_;
    
    // Monitoring thread
    void monitoringThread();
    
    // Calculate packet loss ratio
    double calculateLossRatio() const;
    
    // Calculate RTT from packet timestamps
    double calculateRTT() const;
    
    // Calculate available bandwidth
    double calculateBandwidth() const;
    
    // Calculate frame latency
    double calculateFrameLatency() const;
    
    // Calculate motion entropy from frame data
    double calculateFrameMotionEntropy(const std::vector<uint8_t>& frame_data) const;
    
    // Update metrics with exponential moving average
    void updateMetrics();
    
    // Clean up old frame metrics
    void cleanupOldMetrics();
    
    // Calculate optimal bitrate based on network conditions
    int calculateOptimalBitrate(double bandwidth_mbps, double loss_ratio, double rtt_ms) const;
    
    // Calculate optimal FPS based on frame latency
    int calculateOptimalFPS(double frame_latency_ms) const;
    
    // Calculate optimal resolution based on bandwidth
    std::pair<int, int> calculateOptimalResolution(double bandwidth_mbps) const;
    
    // Calculate maximum bandwidth usage
    double calculateMaxBandwidthUsage(double bandwidth_mbps, double loss_ratio) const;
};

// Global interface functions
void initNetworkMonitor();
void shutdownNetworkMonitor();

// Global network monitor instance
extern NetworkMonitor* g_network_monitor;
NetworkMonitor* getNetworkMonitor(); 