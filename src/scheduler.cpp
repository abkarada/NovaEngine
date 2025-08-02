#include "scheduler.hpp"
#include <numeric>
#include <stdexcept>
#include <iostream>
#include <algorithm>
#include <chrono>

// NovaEngine Ultra Stream - Adaptive Weighted Scheduler
// RTT and loss-based path selection for optimal video delivery

// Global scheduler instance
AdaptiveScheduler* g_scheduler = nullptr;

AdaptiveScheduler::AdaptiveScheduler(const std::vector<PathStats>& paths)
    : paths_(paths), total_weight_(0), running_(false), update_interval_(50) {
    rebuildWeightTable();
}

AdaptiveScheduler::~AdaptiveScheduler() {
    stopMetricCollection();
}

PathStats AdaptiveScheduler::selectPath() {
    if (paths_.empty()) {
        throw std::runtime_error("No paths available for selection");
    }
    
    std::lock_guard<std::mutex> lock(stats_mutex_);
    
    if (total_weight_ <= 0) {
        // Fallback to round-robin if no weights
        static int current_path = 0;
        return paths_[current_path++ % paths_.size()];
    }
    
    // Weighted random selection
    std::uniform_int_distribution<int> dist(1, total_weight_);
    int r = dist(rng_);
    
    for (size_t i = 0; i < cumulative_weights_.size(); ++i) {
        if (r <= cumulative_weights_[i]) {
            stats_.total_chunks_sent++;
            stats_.chunks_per_path[i]++;
            return paths_[i];
        }
    }
    
    // Fallback to last path
    stats_.total_chunks_sent++;
    stats_.chunks_per_path[paths_.size() - 1]++;
    return paths_.back();
}

void AdaptiveScheduler::updateMetrics(const std::vector<PathStats>& new_stats) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    
    paths_ = new_stats;
    
    // Recalculate weights based on new metrics
    for (auto& path : paths_) {
        path.weight = calculateWeight(path.rtt_ms, path.loss_ratio);
    }
    
    rebuildWeightTable();
    updateStatistics();
}

std::vector<PathStats> AdaptiveScheduler::getPathStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return paths_;
}

void AdaptiveScheduler::startMetricCollection() {
    if (running_) return;
    
    running_ = true;
    metric_thread_ = std::thread(&AdaptiveScheduler::metricCollectionThread, this);
    
    std::cout << "[Scheduler] Metric collection started with " << update_interval_.count() << "ms interval" << std::endl;
}

void AdaptiveScheduler::stopMetricCollection() {
    if (!running_) return;
    
    running_ = false;
    if (metric_thread_.joinable()) {
        metric_thread_.join();
    }
    
    std::cout << "[Scheduler] Metric collection stopped" << std::endl;
}

void AdaptiveScheduler::setUpdateInterval(int ms) {
    update_interval_ = std::chrono::milliseconds(ms);
}

AdaptiveScheduler::SchedulerStats AdaptiveScheduler::getStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

void AdaptiveScheduler::rebuildWeightTable() {
    cumulative_weights_.clear();
    total_weight_ = 0;
    
    for (const auto& path : paths_) {
        total_weight_ += path.weight;
        cumulative_weights_.push_back(total_weight_);
    }
    
    std::cout << "[Scheduler] Weight table rebuilt - Total weight: " << total_weight_ << std::endl;
}

void AdaptiveScheduler::metricCollectionThread() {
    while (running_) {
        // Simulate metric collection (in real implementation, this would query RTT/loss monitors)
        std::this_thread::sleep_for(update_interval_);
        
        // Update statistics
        updateStatistics();
        
        // Log current state occasionally
        static int log_counter = 0;
        if (++log_counter % 20 == 0) {  // Every second
            std::lock_guard<std::mutex> lock(stats_mutex_);
            std::cout << "[Scheduler] Active paths: " << paths_.size() 
                     << ", Avg RTT: " << stats_.avg_rtt_ms << "ms"
                     << ", Avg loss: " << (stats_.avg_loss_ratio * 100) << "%"
                     << ", Total chunks: " << stats_.total_chunks_sent << std::endl;
        }
    }
}

int AdaptiveScheduler::calculateWeight(double rtt_ms, double loss_ratio) const {
    // Weight calculation formula:
    // weight = (1000 / (rtt + 1)) * (1 - loss) * bandwidth_factor
    
    // Base weight from RTT (lower RTT = higher weight)
    double rtt_weight = 1000.0 / (rtt_ms + 1.0);
    
    // Loss penalty (higher loss = lower weight)
    double loss_penalty = 1.0 - loss_ratio;
    
    // Combined weight with minimum threshold
    double weight = rtt_weight * loss_penalty;
    
    // Ensure minimum weight of 1
    return std::max(1, static_cast<int>(weight));
}

void AdaptiveScheduler::updateStatistics() {
    if (paths_.empty()) return;
    
    // Calculate average RTT
    double total_rtt = 0.0;
    double total_loss = 0.0;
    
    for (const auto& path : paths_) {
        total_rtt += path.rtt_ms;
        total_loss += path.loss_ratio;
    }
    
    stats_.avg_rtt_ms = total_rtt / paths_.size();
    stats_.avg_loss_ratio = total_loss / paths_.size();
    stats_.last_update = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Global interface functions
void initScheduler(const std::vector<int>& ports) {
    std::vector<PathStats> stats;
    
    for (int port : ports) {
        // Initialize with default metrics (will be updated by monitors)
        stats.emplace_back("127.0.0.1", port, 50.0, 0.0);
    }
    
    if (g_scheduler) {
        delete g_scheduler;
    }
    
    g_scheduler = new AdaptiveScheduler(stats);
    g_scheduler->startMetricCollection();
    
    std::cout << "[Scheduler] ✅ Adaptive scheduler initialized with " << ports.size() << " paths" << std::endl;
}

PathStats selectPathForChunk(int chunk_id) {
    if (!g_scheduler) {
        throw std::runtime_error("Scheduler not initialized");
    }
    
    return g_scheduler->selectPath();
}

void updateSchedulerMetrics(const std::vector<PathStats>& stats) {
    if (g_scheduler) {
        g_scheduler->updateMetrics(stats);
    }
}

void shutdownScheduler() {
    if (g_scheduler) {
        g_scheduler->stopMetricCollection();
        delete g_scheduler;
        g_scheduler = nullptr;
    }
    
    std::cout << "[Scheduler] Scheduler shutdown complete" << std::endl;
}
