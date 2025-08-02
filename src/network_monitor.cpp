#include "network_monitor.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

// NovaEngine Ultra Stream - Dynamic Network Monitoring
// Real-time packet loss, RTT, bandwidth and frame latency calculation

// Global network monitor instance
NetworkMonitor* g_network_monitor = nullptr;

NetworkMonitor::NetworkMonitor() : running_(false) {
    std::cout << "[NetworkMonitor] Initialized" << std::endl;
}

NetworkMonitor::~NetworkMonitor() {
    stopMonitoring();
}

void NetworkMonitor::startMonitoring() {
    if (running_) return;
    
    running_ = true;
    monitor_thread_ = std::thread(&NetworkMonitor::monitoringThread, this);
    
    std::cout << "[NetworkMonitor] Monitoring started" << std::endl;
}

void NetworkMonitor::stopMonitoring() {
    if (!running_) return;
    
    running_ = false;
    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }
    
    std::cout << "[NetworkMonitor] Monitoring stopped" << std::endl;
}

void NetworkMonitor::onPacketSent(uint32_t frame_id, uint16_t chunk_id, uint64_t timestamp, size_t data_size) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    current_metrics_.packets_sent++;
    current_metrics_.bytes_sent += data_size;
    
    // Update frame metrics
    auto& frame_metric = frame_metrics_[frame_id];
    frame_metric.frame_id = frame_id;
    frame_metric.chunks_sent++;
    frame_metric.send_timestamp = timestamp;
}

void NetworkMonitor::onPacketReceived(uint32_t frame_id, uint16_t chunk_id, uint64_t timestamp, size_t data_size) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    current_metrics_.packets_received++;
    current_metrics_.bytes_received += data_size;
    
    // Update frame metrics
    auto& frame_metric = frame_metrics_[frame_id];
    frame_metric.chunks_received++;
    frame_metric.receive_timestamp = timestamp;
    
    // Calculate RTT if we have send timestamp
    if (frame_metric.send_timestamp > 0) {
        double rtt = (timestamp - frame_metric.send_timestamp) / 1000.0; // Convert to ms
        rtt_history_.push_back(rtt);
        
        // Keep only last 100 RTT measurements
        if (rtt_history_.size() > 100) {
            rtt_history_.pop_front();
        }
    }
}

void NetworkMonitor::onPacketLost(uint32_t frame_id, uint16_t chunk_id) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    current_metrics_.packets_lost++;
    
    // Update frame metrics
    auto& frame_metric = frame_metrics_[frame_id];
    frame_metric.chunks_lost++;
}

void NetworkMonitor::onFrameSent(uint32_t frame_id, double frame_size, double motion_entropy, bool is_keyframe) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    auto& frame_metric = frame_metrics_[frame_id];
    frame_metric.frame_id = frame_id;
    frame_metric.frame_size_bytes = frame_size;
    frame_metric.motion_entropy = motion_entropy;
    frame_metric.is_keyframe = is_keyframe;
}

void NetworkMonitor::onFrameReceived(uint32_t frame_id, uint64_t receive_timestamp) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    auto it = frame_metrics_.find(frame_id);
    if (it != frame_metrics_.end()) {
        auto& frame_metric = it->second;
        frame_metric.receive_timestamp = receive_timestamp;
        
        // Calculate frame latency
        if (frame_metric.send_timestamp > 0) {
            double latency = (receive_timestamp - frame_metric.send_timestamp) / 1000.0; // Convert to ms
            latency_history_.push_back(latency);
            
            // Keep only last 100 latency measurements
            if (latency_history_.size() > 100) {
                latency_history_.pop_front();
            }
        }
    }
}

NetworkMetrics NetworkMonitor::getCurrentMetrics() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    return current_metrics_;
}

NetworkMonitor::OptimalParams NetworkMonitor::calculateOptimalParams() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    OptimalParams params;
    
    // Calculate current metrics
    double loss_ratio = calculateLossRatio();
    double rtt_ms = calculateRTT();
    double bandwidth_mbps = calculateBandwidth();
    double frame_latency_ms = calculateFrameLatency();
    
    // Calculate optimal parameters
    params.optimal_bitrate_kbps = calculateOptimalBitrate(bandwidth_mbps, loss_ratio, rtt_ms);
    params.optimal_fps = calculateOptimalFPS(frame_latency_ms);
    auto resolution = calculateOptimalResolution(bandwidth_mbps);
    params.optimal_resolution_width = resolution.first;
    params.optimal_resolution_height = resolution.second;
    params.max_bandwidth_usage = calculateMaxBandwidthUsage(bandwidth_mbps, loss_ratio);
    
    // Calculate compression ratio based on current frame size vs optimal
    double current_frame_size = 640 * 640 * 3; // Current uncompressed size
    params.optimal_compression_ratio = current_frame_size / (params.optimal_bitrate_kbps * 1000.0 / params.optimal_fps);
    
    return params;
}

void NetworkMonitor::setMetricsCallback(std::function<void(const NetworkMetrics&)> callback) {
    metrics_callback_ = callback;
}

double NetworkMonitor::getAverageFrameLatency() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    if (latency_history_.empty()) {
        return 33.0; // Default 30fps latency
    }
    
    double sum = std::accumulate(latency_history_.begin(), latency_history_.end(), 0.0);
    return sum / latency_history_.size();
}

double NetworkMonitor::getFrameLatencyPercentile(double percentile) const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    if (latency_history_.empty()) {
        return 33.0;
    }
    
    std::vector<double> sorted_latencies = std::vector<double>(latency_history_.begin(), latency_history_.end());
    std::sort(sorted_latencies.begin(), sorted_latencies.end());
    
    size_t index = static_cast<size_t>(percentile * sorted_latencies.size() / 100.0);
    return sorted_latencies[index];
}

double NetworkMonitor::calculateMotionEntropy(const std::vector<uint8_t>& frame_data) const {
    // Convert frame data to OpenCV Mat for motion analysis
    if (frame_data.size() < 640 * 640 * 3) {
        return 0.5; // Default medium complexity
    }
    
    cv::Mat frame(640, 640, CV_8UC3, const_cast<uint8_t*>(frame_data.data()));
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    // Calculate motion entropy using Laplacian variance
    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);
    
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    
    // Normalize entropy to 0-1 range
    double entropy = stddev[0] / 255.0;
    return std::min(1.0, std::max(0.0, entropy));
}

double NetworkMonitor::calculateCompressionRatio(double original_size, double compressed_size) const {
    if (compressed_size <= 0) return 1.0;
    return original_size / compressed_size;
}

void NetworkMonitor::monitoringThread() {
    while (running_) {
        updateMetrics();
        cleanupOldMetrics();
        
        // Call callback if set
        if (metrics_callback_) {
            NetworkMetrics metrics = getCurrentMetrics();
            metrics_callback_(metrics);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 50ms update interval
    }
}

double NetworkMonitor::calculateLossRatio() const {
    uint64_t total_packets = current_metrics_.packets_sent + current_metrics_.packets_lost;
    if (total_packets == 0) return 0.0;
    
    double loss_ratio = static_cast<double>(current_metrics_.packets_lost) / total_packets;
    
    // Note: We can't modify loss_history_ in const function, so we'll use a simpler approach
    // In a real implementation, this would be handled differently
    return loss_ratio;
}

double NetworkMonitor::calculateRTT() const {
    if (rtt_history_.empty()) return 50.0; // Default RTT
    
    // Return 95th percentile RTT
    std::vector<double> sorted_rtt = std::vector<double>(rtt_history_.begin(), rtt_history_.end());
    std::sort(sorted_rtt.begin(), sorted_rtt.end());
    
    size_t index = static_cast<size_t>(0.95 * sorted_rtt.size());
    return sorted_rtt[index];
}

double NetworkMonitor::calculateBandwidth() const {
    // Calculate bandwidth based on bytes sent/received over time
    static auto last_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time);
    if (duration.count() == 0) return current_metrics_.bandwidth_mbps;
    
    double time_seconds = duration.count() / 1000.0;
    double bytes_per_second = (current_metrics_.bytes_sent + current_metrics_.bytes_received) / time_seconds;
    double bandwidth_mbps = (bytes_per_second * 8) / (1024 * 1024); // Convert to Mbps
    
    // Apply exponential moving average
    double alpha = 0.4;
    double new_bandwidth = alpha * bandwidth_mbps + (1.0 - alpha) * current_metrics_.bandwidth_mbps;
    
    last_time = current_time;
    return new_bandwidth;
}

double NetworkMonitor::calculateFrameLatency() const {
    if (latency_history_.empty()) return 33.0; // Default 30fps latency
    
    // Return 95th percentile latency
    std::vector<double> sorted_latency = std::vector<double>(latency_history_.begin(), latency_history_.end());
    std::sort(sorted_latency.begin(), sorted_latency.end());
    
    size_t index = static_cast<size_t>(0.95 * sorted_latency.size());
    return sorted_latency[index];
}

void NetworkMonitor::updateMetrics() {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    // Update current metrics with calculated values
    current_metrics_.loss_ratio = calculateLossRatio();
    current_metrics_.rtt_ms = calculateRTT();
    current_metrics_.bandwidth_mbps = calculateBandwidth();
    current_metrics_.frame_latency_ms = calculateFrameLatency();
    current_metrics_.last_update = std::chrono::steady_clock::now();
    
    // Calculate jitter from RTT history
    if (rtt_history_.size() > 1) {
        std::vector<double> rtt_copy(rtt_history_.begin(), rtt_history_.end());
        std::sort(rtt_copy.begin(), rtt_copy.end());
        
        double median = rtt_copy[rtt_copy.size() / 2];
        double sum_deviation = 0.0;
        for (double rtt : rtt_copy) {
            sum_deviation += std::abs(rtt - median);
        }
        current_metrics_.jitter_ms = sum_deviation / rtt_copy.size();
    }
}

void NetworkMonitor::cleanupOldMetrics() {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    // Remove frame metrics older than 5 seconds
    auto cutoff_time = std::chrono::steady_clock::now() - std::chrono::seconds(5);
    
    for (auto it = frame_metrics_.begin(); it != frame_metrics_.end();) {
        if (it->second.receive_timestamp > 0) {
            auto frame_time = std::chrono::steady_clock::time_point(std::chrono::microseconds(it->second.receive_timestamp));
            if (frame_time < cutoff_time) {
                it = frame_metrics_.erase(it);
            } else {
                ++it;
            }
        } else {
            ++it;
        }
    }
}

int NetworkMonitor::calculateOptimalBitrate(double bandwidth_mbps, double loss_ratio, double rtt_ms) const {
    // Advanced bitrate calculation formula
    // Optimal bitrate = network_capacity * efficiency_factor * quality_factor * latency_factor
    
    // Base efficiency (use 80% of available bandwidth)
    double efficiency_factor = 0.80;
    
    // Quality factor based on loss ratio
    double quality_factor = 1.0;
    if (loss_ratio > 0.01) {  // >1% loss
        quality_factor = 1.0 - (loss_ratio * 8.0);  // Reduce bitrate for high loss
    }
    
    // Latency factor based on RTT
    double latency_factor = 1.0;
    if (rtt_ms > 50.0) {  // >50ms RTT
        latency_factor = 1.0 - ((rtt_ms - 50.0) / 300.0);  // Reduce for high latency
    }
    
    // Calculate optimal bitrate
    double optimal_bitrate = bandwidth_mbps * 1000 * efficiency_factor * quality_factor * latency_factor;
    
    // Apply bitrate tiers with quality preservation
    if (optimal_bitrate > 8000) {
        return 8000;  // 8 Mbps for excellent networks
    } else if (optimal_bitrate > 4000) {
        return 4000;  // 4 Mbps for good networks
    } else if (optimal_bitrate > 2000) {
        return 2500;  // 2.5 Mbps for medium networks
    } else if (optimal_bitrate > 1000) {
        return 1500;  // 1.5 Mbps for poor networks
    } else {
        return 1000;  // 1 Mbps minimum
    }
}

int NetworkMonitor::calculateOptimalFPS(double frame_latency_ms) const {
    // FPS calculation based on frame latency
    // Max FPS = 1000 / Total Frame Latency
    double max_fps = 1000.0 / frame_latency_ms;
    
    // Apply FPS tiers
    if (max_fps >= 30.0) {
        return 30;  // 30fps for excellent conditions
    } else if (max_fps >= 25.0) {
        return 25;  // 25fps for good conditions
    } else if (max_fps >= 20.0) {
        return 20;  // 20fps for medium conditions
    } else if (max_fps >= 15.0) {
        return 15;  // 15fps for poor conditions
    } else {
        return 10;  // 10fps minimum
    }
}

std::pair<int, int> NetworkMonitor::calculateOptimalResolution(double bandwidth_mbps) const {
    // Resolution calculation based on bandwidth
    // Frame size = resolution * bit_depth * compression_factor
    
    if (bandwidth_mbps >= 8.0) {
        return {1280, 720};  // 720p for excellent networks
    } else if (bandwidth_mbps >= 4.0) {
        return {640, 640};   // 640x640 for good networks
    } else if (bandwidth_mbps >= 2.0) {
        return {480, 480};   // 480x480 for medium networks
    } else {
        return {320, 320};   // 320x320 for poor networks
    }
}

double NetworkMonitor::calculateMaxBandwidthUsage(double bandwidth_mbps, double loss_ratio) const {
    // Calculate maximum safe bandwidth usage
    double base_usage = 0.80;  // 80% base usage
    
    // Reduce usage for high loss networks
    if (loss_ratio > 0.05) {  // >5% loss
        base_usage *= (1.0 - loss_ratio * 2.0);
    }
    
    return std::min(0.95, std::max(0.50, base_usage));  // Between 50% and 95%
}

// Global interface functions
void initNetworkMonitor() {
    if (g_network_monitor) {
        delete g_network_monitor;
    }
    
    g_network_monitor = new NetworkMonitor();
    g_network_monitor->startMonitoring();
    std::cout << "[NetworkMonitor] ✅ Global network monitor initialized" << std::endl;
}

void shutdownNetworkMonitor() {
    if (g_network_monitor) {
        delete g_network_monitor;
        g_network_monitor = nullptr;
    }
    
    std::cout << "[NetworkMonitor] Network monitor shutdown complete" << std::endl;
}

NetworkMonitor* getNetworkMonitor() {
    return g_network_monitor;
} 