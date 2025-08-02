#include "erasure_coder.hpp"
#include <jerasure.h>
#include <reed_sol.h>
#include <iostream>
#include <chrono>
#include <cstring>
#include <algorithm>

// NovaEngine Ultra Stream - True Reed-Solomon FEC
// High-performance erasure coding with libjerasure

// Global erasure coder instance
ErasureCoder* g_erasure_coder = nullptr;

ErasureCoder::ErasureCoder(int k_chunks, int r_chunks, int chunk_size)
    : k_(k_chunks), r_(r_chunks), chunk_size_(chunk_size), 
      rs_matrix_(nullptr), rs_bitmatrix_(nullptr) {
    
    if (!initMatrices()) {
        throw std::runtime_error("Failed to initialize Reed-Solomon matrices");
    }
    
    std::cout << "[ErasureCoder] Initialized with k=" << k_ << ", r=" << r_ 
              << ", chunk_size=" << chunk_size_ << std::endl;
}

ErasureCoder::~ErasureCoder() {
    cleanupMatrices();
}

std::vector<std::vector<uint8_t>> ErasureCoder::encode(const std::vector<uint8_t>& data) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Pad data to fit exactly k chunks
    std::vector<uint8_t> padded_data = padData(data);
    
    // Prepare data pointers for jerasure
    char** data_ptrs = new char*[k_];
    char** coding_ptrs = new char*[r_];
    
    for (int i = 0; i < k_; ++i) {
        data_ptrs[i] = new char[chunk_size_];
        memcpy(data_ptrs[i], padded_data.data() + i * chunk_size_, chunk_size_);
    }
    
    for (int i = 0; i < r_; ++i) {
        coding_ptrs[i] = new char[chunk_size_];
        memset(coding_ptrs[i], 0, chunk_size_);
    }
    
    // Perform Reed-Solomon encoding
    jerasure_matrix_encode(k_, r_, 8, rs_matrix_, data_ptrs, coding_ptrs, chunk_size_);
    
    // Collect all chunks (data + parity)
    std::vector<std::vector<uint8_t>> chunks;
    
    // Add data chunks
    for (int i = 0; i < k_; ++i) {
        std::vector<uint8_t> chunk(data_ptrs[i], data_ptrs[i] + chunk_size_);
        chunks.push_back(chunk);
    }
    
    // Add parity chunks
    for (int i = 0; i < r_; ++i) {
        std::vector<uint8_t> chunk(coding_ptrs[i], coding_ptrs[i] + chunk_size_);
        chunks.push_back(chunk);
    }
    
    // Cleanup
    for (int i = 0; i < k_; ++i) {
        delete[] data_ptrs[i];
    }
    for (int i = 0; i < r_; ++i) {
        delete[] coding_ptrs[i];
    }
    delete[] data_ptrs;
    delete[] coding_ptrs;
    
    // Update statistics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    stats_.frames_encoded++;
    stats_.total_encode_time_us += duration.count();
    
    std::cout << "[ErasureCoder] Encoded " << data.size() << " bytes into " 
              << chunks.size() << " chunks in " << duration.count() << "μs" << std::endl;
    
    return chunks;
}

std::vector<uint8_t> ErasureCoder::decode(const std::vector<std::vector<uint8_t>>& chunks,
                                         const std::vector<bool>& chunk_available) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    if (!canDecode(chunk_available)) {
        stats_.frames_lost++;
        return {};
    }
    
    // Prepare data pointers for jerasure
    char** data_ptrs = new char*[k_ + r_];
    char** coding_ptrs = new char*[r_];
    
    for (int i = 0; i < k_ + r_; ++i) {
        data_ptrs[i] = new char[chunk_size_];
        if (chunk_available[i]) {
            memcpy(data_ptrs[i], chunks[i].data(), chunk_size_);
        } else {
            memset(data_ptrs[i], 0, chunk_size_);
        }
    }
    
    for (int i = 0; i < r_; ++i) {
        coding_ptrs[i] = data_ptrs[k_ + i];
    }
    
    // Create erasures list
    std::vector<int> erasures = createErasuresList(chunk_available);
    
    // Perform Reed-Solomon decoding
    int decode_result = jerasure_matrix_decode(k_, r_, 8, rs_matrix_, 1, 
                                             erasures.data(), data_ptrs, coding_ptrs, chunk_size_);
    
    std::vector<uint8_t> decoded_data;
    
    if (decode_result == 0) {
        // Successfully decoded
        for (int i = 0; i < k_; ++i) {
            decoded_data.insert(decoded_data.end(), 
                              reinterpret_cast<uint8_t*>(data_ptrs[i]),
                              reinterpret_cast<uint8_t*>(data_ptrs[i]) + chunk_size_);
        }
        
        stats_.frames_decoded++;
        if (erasures.size() > 0) {
            stats_.frames_recovered++;
        }
        
        std::cout << "[ErasureCoder] Successfully decoded frame with " 
                  << erasures.size() << " erasures" << std::endl;
    } else {
        stats_.frames_lost++;
        std::cout << "[ErasureCoder] Failed to decode frame (result: " << decode_result << ")" << std::endl;
    }
    
    // Cleanup
    for (int i = 0; i < k_ + r_; ++i) {
        delete[] data_ptrs[i];
    }
    delete[] data_ptrs;
    
    // Update statistics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    stats_.total_decode_time_us += duration.count();
    
    return decoded_data;
}

bool ErasureCoder::canDecode(const std::vector<bool>& chunk_available) const {
    if (chunk_available.size() != k_ + r_) {
        return false;
    }
    
    int available_count = 0;
    for (bool available : chunk_available) {
        if (available) available_count++;
    }
    
    return available_count >= k_;
}

ErasureCoder::CodingStats ErasureCoder::getStats() const {
    CodingStats stats = stats_;
    
    // Calculate recovery rate
    if (stats.frames_decoded > 0) {
        stats.recovery_rate = static_cast<double>(stats.frames_recovered) / stats.frames_decoded;
    }
    
    return stats;
}

void ErasureCoder::setParams(int k, int r, int chunk_size) {
    cleanupMatrices();
    
    k_ = k;
    r_ = r;
    chunk_size_ = chunk_size;
    
    if (!initMatrices()) {
        throw std::runtime_error("Failed to reinitialize Reed-Solomon matrices");
    }
    
    std::cout << "[ErasureCoder] Parameters updated: k=" << k_ << ", r=" << r_ 
              << ", chunk_size=" << chunk_size_ << std::endl;
}

bool ErasureCoder::initMatrices() {
    // Initialize Reed-Solomon encoding matrix
    rs_matrix_ = reed_sol_vandermonde_coding_matrix(k_, r_, 8);
    if (!rs_matrix_) {
        std::cerr << "[ErasureCoder] Failed to create Reed-Solomon matrix" << std::endl;
        return false;
    }
    
    // Create bit matrix for faster operations
    rs_bitmatrix_ = jerasure_matrix_to_bitmatrix(k_, r_, 8, rs_matrix_);
    if (!rs_bitmatrix_) {
        std::cerr << "[ErasureCoder] Failed to create bit matrix" << std::endl;
        return false;
    }
    
    return true;
}

void ErasureCoder::cleanupMatrices() {
    if (rs_matrix_) {
        free(rs_matrix_);
        rs_matrix_ = nullptr;
    }
    
    if (rs_bitmatrix_) {
        free(rs_bitmatrix_);
        rs_bitmatrix_ = nullptr;
    }
}

std::vector<uint8_t> ErasureCoder::padData(const std::vector<uint8_t>& data) {
    int target_size = k_ * chunk_size_;
    std::vector<uint8_t> padded = data;
    
    if (padded.size() < static_cast<size_t>(target_size)) {
        // Pad with zeros
        padded.resize(target_size, 0);
    } else if (padded.size() > static_cast<size_t>(target_size)) {
        // Truncate
        padded.resize(target_size);
    }
    
    return padded;
}

std::vector<uint8_t> ErasureCoder::unpadData(const std::vector<uint8_t>& data, size_t original_size) {
    if (data.size() <= original_size) {
        return data;
    }
    
    return std::vector<uint8_t>(data.begin(), data.begin() + original_size);
}

std::vector<int> ErasureCoder::createErasuresList(const std::vector<bool>& chunk_available) {
    std::vector<int> erasures;
    
    for (size_t i = 0; i < chunk_available.size(); ++i) {
        if (!chunk_available[i]) {
            erasures.push_back(static_cast<int>(i));
        }
    }
    
    // Add -1 terminator
    erasures.push_back(-1);
    
    return erasures;
}

// Global interface functions
void initErasureCoder(int k, int r, int chunk_size) {
    if (g_erasure_coder) {
        delete g_erasure_coder;
    }
    
    g_erasure_coder = new ErasureCoder(k, r, chunk_size);
    std::cout << "[ErasureCoder] ✅ Global erasure coder initialized" << std::endl;
}

std::vector<std::vector<uint8_t>> encodeFrame(const std::vector<uint8_t>& data) {
    if (!g_erasure_coder) {
        throw std::runtime_error("Erasure coder not initialized");
    }
    
    return g_erasure_coder->encode(data);
}

std::vector<uint8_t> decodeFrame(const std::vector<std::vector<uint8_t>>& chunks,
                                const std::vector<bool>& chunk_available) {
    if (!g_erasure_coder) {
        throw std::runtime_error("Erasure coder not initialized");
    }
    
    return g_erasure_coder->decode(chunks, chunk_available);
}

void shutdownErasureCoder() {
    if (g_erasure_coder) {
        delete g_erasure_coder;
        g_erasure_coder = nullptr;
    }
    
    std::cout << "[ErasureCoder] Erasure coder shutdown complete" << std::endl;
}

