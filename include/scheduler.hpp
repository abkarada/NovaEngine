#pragma once

#include "udp_sender.hpp"
#include <vector>
#include <map>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <deque>

// NovaEngine Ultra Stream - Advanced Path Scoring & Weighted Scheduler
// Real-time RTT/Loss based path optimization with redundant slice distribution

struct PathMetrics {
    double rtt_ms;              // Round-trip time in milliseconds
    double loss_ratio;          // Packet loss ratio (0.0-1.0)
    double throughput_mbps;     // Measured throughput
    double jitter_ms;           // Jitter measurement
    uint64_t packets_sent;      // Total packets sent
    uint64_t packets_lost;      // Total packets lost
    uint64_t packets_received;  // Total packets received
    std::chrono::steady_clock::time_point last_update;
    
    PathMetrics() : rtt_ms(100.0), loss_ratio(0.1), throughput_mbps(10.0), 
                   jitter_ms(5.0), packets_sent(0), packets_lost(0), packets_received(0) {
        last_update = std::chrono::steady_clock::now();
    }
};

struct PathScore {
    int path_id;
    double score;               // 0.0-100.0 (higher is better)
    double weight;              // Scheduler weight
    bool is_primary;            // Primary path for data chunks
    bool is_redundant;          // Redundant path for parity chunks
    
    PathScore() : path_id(0), score(0.0), weight(0.0), is_primary(false), is_redundant(false) {}
};

class AdaptiveScheduler {
public:
    AdaptiveScheduler(const std::vector<int>& ports);
    ~AdaptiveScheduler();
    
    // Path selection for chunks
    int selectPathForChunk(uint32_t frame_id, uint16_t chunk_id, bool is_parity);
    
    // Update path metrics
    void updatePathMetrics(int path_id, double rtt_ms, double loss_ratio, double throughput_mbps);
    
    // Get scheduler statistics
    struct SchedulerStats {
        uint64_t total_chunks_sent;
        std::map<int, uint64_t> chunks_per_path;
        double avg_rtt_ms;
        double avg_loss_ratio;
        double avg_throughput_mbps;
        std::vector<PathScore> path_scores;
    };
    
    SchedulerStats getStats() const;
    
    // Start metric collection thread
    void startMetricCollection();
    
    // Stop metric collection
    void stopMetricCollection();
    
    // Set FEC parameters for slice distribution
    void setFECParams(int k_chunks, int r_chunks);
    
    // Get optimal path distribution
    std::vector<int> getOptimalPathDistribution(uint32_t frame_id);

private:
    std::vector<int> ports_;
    std::map<int, PathMetrics> path_metrics_;
    std::map<int, PathScore> path_scores_;
    mutable std::mutex metrics_mutex_;
    std::atomic<bool> running_;
    std::thread metric_thread_;
    SchedulerStats stats_;
    int k_chunks_;
    int r_chunks_;
    
    // Path scoring algorithm
    double calculatePathScore(const PathMetrics& metrics) const;
    
    // Weight calculation based on score
    double calculateWeight(double score) const;
    
    // Update path scores
    void updatePathScores();
    
    // Metric collection thread
    void metricCollectionThread();
    
    // Slice distribution strategy
    int selectPrimaryPath(uint32_t frame_id, uint16_t chunk_id) const;
    int selectRedundantPath(uint32_t frame_id, uint16_t chunk_id) const;
    
    // Load balancing
    double getPathLoad(int path_id) const;
    void updatePathLoad(int path_id);
    
    // Exponential moving average for smoothing
    double calculateEMA(double current, double previous, double alpha = 0.3) const;
};

// Global scheduler interface
extern AdaptiveScheduler* g_scheduler;

void initScheduler(const std::vector<int>& ports);
int selectPathForChunk(uint32_t frame_id, uint16_t chunk_id, bool is_parity);
void updateSchedulerMetrics(const std::vector<PathStats>& path_stats);
void shutdownScheduler();
