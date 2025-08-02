#pragma once

#include "udp_sender.hpp"
#include <vector>
#include <random>
#include <chrono>
#include <atomic>
#include <thread>
#include <mutex>

// NovaEngine Ultra Stream - Adaptive Weighted Scheduler
// RTT and loss-based path selection for optimal video delivery

class AdaptiveScheduler {
public:
    AdaptiveScheduler(const std::vector<PathStats>& paths);
    ~AdaptiveScheduler();
    
    // Select optimal path for chunk transmission
    PathStats selectPath();
    
    // Update path metrics (RTT, loss, bandwidth)
    void updateMetrics(const std::vector<PathStats>& new_stats);
    
    // Get current path statistics
    std::vector<PathStats> getPathStats() const;
    
    // Start background metric collection
    void startMetricCollection();
    
    // Stop metric collection
    void stopMetricCollection();
    
    // Set metric update interval (default: 50ms)
    void setUpdateInterval(int ms);
    
    // Get scheduler statistics
    struct SchedulerStats {
        uint64_t total_chunks_sent;
        uint64_t chunks_per_path[16];  // Max 16 paths
        double avg_rtt_ms;
        double avg_loss_ratio;
        uint64_t last_update;
    };
    
    SchedulerStats getStats() const;

private:
    std::vector<PathStats> paths_;
    std::vector<int> cumulative_weights_;
    int total_weight_;
    std::mt19937 rng_;
    std::atomic<bool> running_;
    std::thread metric_thread_;
    std::chrono::milliseconds update_interval_;
    mutable std::mutex stats_mutex_;
    SchedulerStats stats_;
    
    // Rebuild weight table based on current metrics
    void rebuildWeightTable();
    
    // Background metric collection thread
    void metricCollectionThread();
    
    // Calculate path weight based on RTT and loss
    int calculateWeight(double rtt_ms, double loss_ratio) const;
    
    // Update internal statistics
    void updateStatistics();
};

// Global scheduler interface
extern AdaptiveScheduler* g_scheduler;

void initScheduler(const std::vector<int>& ports);
PathStats selectPathForChunk(int chunk_id);
void updateSchedulerMetrics(const std::vector<PathStats>& stats);
void shutdownScheduler();
