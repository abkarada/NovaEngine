#include "scheduler.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <random>

// NovaEngine Ultra Stream - Advanced Path Scoring & Weighted Scheduler
// Real-time RTT/Loss based path optimization with redundant slice distribution

// Global scheduler instance
AdaptiveScheduler* g_scheduler = nullptr;

AdaptiveScheduler::AdaptiveScheduler(const std::vector<int>& ports)
    : ports_(ports), running_(false), k_chunks_(6), r_chunks_(2) {
    
    // Initialize path metrics for each port
    for (int port : ports) {
        path_metrics_[port] = PathMetrics();
        path_scores_[port] = PathScore();
        path_scores_[port].path_id = port;
        stats_.chunks_per_path[port] = 0;
    }
    
    std::cout << "[Scheduler] Initialized with " << ports.size() << " paths" << std::endl;
}

AdaptiveScheduler::~AdaptiveScheduler() {
    stopMetricCollection();
}

int AdaptiveScheduler::selectPathForChunk(uint32_t frame_id, uint16_t chunk_id, bool is_parity) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    int selected_path;
    
    if (is_parity) {
        // Parity chunks go to redundant paths
        selected_path = selectRedundantPath(frame_id, chunk_id);
    } else {
        // Data chunks go to primary paths
        selected_path = selectPrimaryPath(frame_id, chunk_id);
    }
    
    // Update statistics
    stats_.total_chunks_sent++;
    stats_.chunks_per_path[selected_path]++;
    updatePathLoad(selected_path);
    
    return selected_path;
}

double AdaptiveScheduler::calculatePathScore(const PathMetrics& metrics) const {
    // Advanced path scoring formula
    // Score = (Throughput * (1 - Loss) * (1 / RTT) * (1 / Jitter)) * 100
    
    // Normalize metrics to 0-1 range
    double throughput_score = std::min(metrics.throughput_mbps / 100.0, 1.0);  // Max 100 Mbps
    double loss_score = 1.0 - metrics.loss_ratio;  // Lower loss = higher score
    double rtt_score = std::max(0.0, 1.0 - (metrics.rtt_ms / 1000.0));  // Max 1000ms RTT
    double jitter_score = std::max(0.0, 1.0 - (metrics.jitter_ms / 100.0));  // Max 100ms jitter
    
    // Calculate composite score
    double score = throughput_score * loss_score * rtt_score * jitter_score * 100.0;
    
    // Apply exponential penalty for high loss
    if (metrics.loss_ratio > 0.05) {  // >5% loss
        score *= std::exp(-metrics.loss_ratio * 10.0);
    }
    
    // Apply RTT penalty for high latency
    if (metrics.rtt_ms > 100.0) {  // >100ms RTT
        score *= std::exp(-(metrics.rtt_ms - 100.0) / 200.0);
    }
    
    return std::max(0.0, std::min(100.0, score));
}

double AdaptiveScheduler::calculateWeight(double score) const {
    // Convert score to weight using exponential function
    // Higher score = exponentially higher weight
    return std::pow(score / 50.0, 2.0) + 1.0;
}

void AdaptiveScheduler::updatePathScores() {
    for (auto& pair : path_metrics_) {
        int path_id = pair.first;
        const PathMetrics& metrics = pair.second;
        
        PathScore& score = path_scores_[path_id];
        score.path_id = path_id;
        score.score = calculatePathScore(metrics);
        score.weight = calculateWeight(score.score);
        
        // Determine path roles based on score
        score.is_primary = (score.score > 50.0);  // High score = primary path
        score.is_redundant = (score.score > 20.0);  // Medium score = redundant path
    }
    
    // Update global statistics
    double total_rtt = 0.0, total_loss = 0.0, total_throughput = 0.0;
    int active_paths = 0;
    
    for (const auto& pair : path_metrics_) {
        const PathMetrics& metrics = pair.second;
        total_rtt += metrics.rtt_ms;
        total_loss += metrics.loss_ratio;
        total_throughput += metrics.throughput_mbps;
        active_paths++;
    }
    
    if (active_paths > 0) {
        stats_.avg_rtt_ms = total_rtt / active_paths;
        stats_.avg_loss_ratio = total_loss / active_paths;
        stats_.avg_throughput_mbps = total_throughput / active_paths;
    }
    
    // Update path scores in stats
    stats_.path_scores.clear();
    for (const auto& pair : path_scores_) {
        stats_.path_scores.push_back(pair.second);
    }
}

int AdaptiveScheduler::selectPrimaryPath(uint32_t frame_id, uint16_t chunk_id) const {
    // Select best primary path for data chunks
    std::vector<std::pair<int, double>> candidates;
    
    for (const auto& pair : path_scores_) {
        const PathScore& score = pair.second;
        if (score.is_primary) {
            // Consider both score and current load
            double load_factor = 1.0 / (1.0 + getPathLoad(score.path_id));
            double weighted_score = score.score * load_factor;
            candidates.emplace_back(score.path_id, weighted_score);
        }
    }
    
    if (candidates.empty()) {
        // Fallback to any available path
        for (const auto& pair : path_scores_) {
            candidates.emplace_back(pair.first, pair.second.score);
        }
    }
    
    if (candidates.empty()) {
        return ports_[0];  // Fallback to first port
    }
    
    // Sort by weighted score (descending)
    std::sort(candidates.begin(), candidates.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });
    
    // Use weighted random selection from top 3 candidates
    int num_candidates = std::min(3, static_cast<int>(candidates.size()));
    double total_weight = 0.0;
    
    for (int i = 0; i < num_candidates; i++) {
        total_weight += candidates[i].second;
    }
    
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, total_weight);
    
    double random_value = dis(gen);
    double cumulative_weight = 0.0;
    
    for (int i = 0; i < num_candidates; i++) {
        cumulative_weight += candidates[i].second;
        if (random_value <= cumulative_weight) {
            return candidates[i].first;
        }
    }
    
    return candidates[0].first;  // Fallback
}

int AdaptiveScheduler::selectRedundantPath(uint32_t frame_id, uint16_t chunk_id) const {
    // Select different path for parity chunks to ensure redundancy
    std::vector<std::pair<int, double>> candidates;
    
    for (const auto& pair : path_scores_) {
        const PathScore& score = pair.second;
        if (score.is_redundant) {
            // Prefer paths with lower load for redundancy
            double load_factor = 1.0 / (1.0 + getPathLoad(score.path_id));
            double weighted_score = score.score * load_factor;
            candidates.emplace_back(score.path_id, weighted_score);
        }
    }
    
    if (candidates.empty()) {
        // Fallback to any available path
        for (const auto& pair : path_scores_) {
            candidates.emplace_back(pair.first, pair.second.score);
        }
    }
    
    if (candidates.empty()) {
        return ports_[0];
    }
    
    // Sort by weighted score (descending)
    std::sort(candidates.begin(), candidates.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });
    
    // Select path with lowest load among top candidates
    int selected_path = candidates[0].first;
    double min_load = getPathLoad(selected_path);
    
    for (const auto& candidate : candidates) {
        double load = getPathLoad(candidate.first);
        if (load < min_load) {
            min_load = load;
            selected_path = candidate.first;
        }
    }
    
    return selected_path;
}

double AdaptiveScheduler::getPathLoad(int path_id) const {
    auto it = stats_.chunks_per_path.find(path_id);
    if (it == stats_.chunks_per_path.end()) {
        return 0.0;
    }
    
    // Calculate load as chunks sent in last 1000 chunks
    return static_cast<double>(it->second) / std::max(1.0, static_cast<double>(stats_.total_chunks_sent));
}

void AdaptiveScheduler::updatePathLoad(int path_id) {
    // Load is automatically updated in selectPathForChunk
}

double AdaptiveScheduler::calculateEMA(double current, double previous, double alpha) const {
    return alpha * current + (1.0 - alpha) * previous;
}

void AdaptiveScheduler::updatePathMetrics(int path_id, double rtt_ms, double loss_ratio, double throughput_mbps) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    auto it = path_metrics_.find(path_id);
    if (it == path_metrics_.end()) {
        return;
    }
    
    PathMetrics& metrics = it->second;
    
    // Apply exponential moving average for smoothing
    metrics.rtt_ms = calculateEMA(rtt_ms, metrics.rtt_ms, 0.3);
    metrics.loss_ratio = calculateEMA(loss_ratio, metrics.loss_ratio, 0.2);
    metrics.throughput_mbps = calculateEMA(throughput_mbps, metrics.throughput_mbps, 0.4);
    
    metrics.last_update = std::chrono::steady_clock::now();
    
    // Update path scores
    updatePathScores();
    
    std::cout << "[Scheduler] Path " << path_id << " updated: RTT=" << metrics.rtt_ms 
              << "ms, Loss=" << (metrics.loss_ratio * 100) << "%, Throughput=" 
              << metrics.throughput_mbps << "Mbps, Score=" << path_scores_[path_id].score << std::endl;
}

AdaptiveScheduler::SchedulerStats AdaptiveScheduler::getStats() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    return stats_;
}

void AdaptiveScheduler::startMetricCollection() {
    if (running_) return;
    
    running_ = true;
    metric_thread_ = std::thread(&AdaptiveScheduler::metricCollectionThread, this);
    
    std::cout << "[Scheduler] Metric collection started with 50ms interval" << std::endl;
}

void AdaptiveScheduler::stopMetricCollection() {
    if (!running_) return;
    
    running_ = false;
    if (metric_thread_.joinable()) {
        metric_thread_.join();
    }
    
    std::cout << "[Scheduler] Metric collection stopped" << std::endl;
}

void AdaptiveScheduler::setFECParams(int k_chunks, int r_chunks) {
    k_chunks_ = k_chunks;
    r_chunks_ = r_chunks;
    std::cout << "[Scheduler] FEC params set: k=" << k_chunks_ << ", r=" << r_chunks_ << std::endl;
}

std::vector<int> AdaptiveScheduler::getOptimalPathDistribution(uint32_t frame_id) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    std::vector<int> distribution;
    distribution.reserve(k_chunks_ + r_chunks_);
    
    // Distribute data chunks across primary paths
    for (int i = 0; i < k_chunks_; i++) {
        int path = selectPrimaryPath(frame_id, i);
        distribution.push_back(path);
    }
    
    // Distribute parity chunks across redundant paths
    for (int i = 0; i < r_chunks_; i++) {
        int path = selectRedundantPath(frame_id, k_chunks_ + i);
        distribution.push_back(path);
    }
    
    return distribution;
}

void AdaptiveScheduler::metricCollectionThread() {
    while (running_) {
        // Rebuild weight table periodically
        {
            std::lock_guard<std::mutex> lock(metrics_mutex_);
            updatePathScores();
        }
        
        // Print statistics every 5 seconds
        static int counter = 0;
        if (++counter % 100 == 0) {  // 100 * 50ms = 5 seconds
            std::cout << "[Scheduler] Active paths: " << path_scores_.size() 
                      << ", Avg RTT: " << stats_.avg_rtt_ms << "ms, Avg loss: " 
                      << (stats_.avg_loss_ratio * 100) << "%, Total chunks: " 
                      << stats_.total_chunks_sent << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

// Global interface functions
void initScheduler(const std::vector<int>& ports) {
    if (g_scheduler) {
        delete g_scheduler;
    }
    
    g_scheduler = new AdaptiveScheduler(ports);
    g_scheduler->startMetricCollection();
    std::cout << "[Scheduler] ✅ Adaptive scheduler initialized" << std::endl;
}

int selectPathForChunk(uint32_t frame_id, uint16_t chunk_id, bool is_parity) {
    if (!g_scheduler) {
        throw std::runtime_error("Scheduler not initialized");
    }
    
    return g_scheduler->selectPathForChunk(frame_id, chunk_id, is_parity);
}

void updateSchedulerMetrics(const std::vector<PathStats>& path_stats) {
    if (!g_scheduler) return;
    
    for (const auto& stat : path_stats) {
        g_scheduler->updatePathMetrics(stat.port, stat.rtt_ms, stat.loss_ratio, stat.throughput_mbps);
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
