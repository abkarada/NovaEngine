#include "erasure_coder.hpp"
#include <stdexcept>
#include <jerasure.h>
#include <reed_sol.h>
#include <cstring>
#include <vector>
#include <iostream>

ErasureCoder::ErasureCoder(int k, int r, int w)
    : k_(k), r_(r), w_(w) {
    matrix_ = reed_sol_vandermonde_coding_matrix(k_, r_, w_);
    if (!matrix_) throw std::runtime_error("Failed to create coding matrix");
}

ErasureCoder::~ErasureCoder() {
    if (matrix_) free(matrix_);
}

void ErasureCoder::encode(const std::vector<std::vector<uint8_t>>& k_chunks,
                          std::vector<std::vector<uint8_t>>& out_blocks) {
    if (k_chunks.size() != static_cast<size_t>(k_))
        throw std::runtime_error("Invalid number of data chunks");

    size_t block_size = k_chunks[0].size();
    
    // Initialize output blocks with data chunks
    out_blocks.clear();
    out_blocks.reserve(k_ + r_);
    
    // Copy data chunks
    for (const auto& chunk : k_chunks) {
        out_blocks.push_back(chunk);
    }
    
    // Add parity chunks (initialized to zero)
    for (int i = 0; i < r_; ++i) {
        out_blocks.push_back(std::vector<uint8_t>(block_size, 0));
    }

    std::vector<uint8_t*> data_ptrs(k_);
    std::vector<uint8_t*> code_ptrs(r_);
    for (int i = 0; i < k_; ++i) data_ptrs[i] = const_cast<uint8_t*>(k_chunks[i].data());
    for (int i = 0; i < r_; ++i) code_ptrs[i] = out_blocks[k_ + i].data();

    std::cout << "[FEC] Encoding " << k_ << " data chunks + " << r_ << " parity chunks, block_size=" << block_size << std::endl;

    jerasure_matrix_encode(k_, r_, w_, matrix_,
                           reinterpret_cast<char**>(data_ptrs.data()),
                           reinterpret_cast<char**>(code_ptrs.data()),
                           block_size);
                           
    std::cout << "[FEC] Encoded " << out_blocks.size() << " total chunks" << std::endl;
}

bool ErasureCoder::decode(const std::vector<std::vector<uint8_t>>& blocks,
                          const std::vector<bool>& received,
                          std::vector<uint8_t>& recovered_data) {
    if (blocks.size() != static_cast<size_t>(k_ + r_)) {
        std::cerr << "[FEC] Invalid blocks size: " << blocks.size() << " != " << (k_ + r_) << std::endl;
        return false;
    }
    if (received.size() != blocks.size()) {
        std::cerr << "[FEC] Invalid received flags size: " << received.size() << " != " << blocks.size() << std::endl;
        return false;
    }

    size_t block_size = blocks[0].size();
    
    // Count received blocks
    int received_count = 0;
    for (bool r : received) if (r) received_count++;
    
    if (received_count < k_) {
        std::cerr << "[FEC] Not enough blocks received: " << received_count << " < " << k_ << std::endl;
        return false;
    }

    // Create working copies of the blocks
    std::vector<std::vector<uint8_t>> working_blocks = blocks;
    
    // Prepare data and parity pointers
    std::vector<uint8_t*> data_ptrs(k_);
    std::vector<uint8_t*> code_ptrs(r_);
    
    for (int i = 0; i < k_; ++i) {
        data_ptrs[i] = working_blocks[i].data();
    }
    for (int i = 0; i < r_; ++i) {
        code_ptrs[i] = working_blocks[k_ + i].data();
    }

    // Build erasure list
    std::vector<int> erasures;
    for (int i = 0; i < k_ + r_; ++i) {
        if (!received[i]) {
            erasures.push_back(i);
        }
    }
    erasures.push_back(-1); // terminator

    // If no erasures, just return the data blocks
    if (erasures.size() == 1) {
        recovered_data.clear();
        for (int i = 0; i < k_; ++i) {
            recovered_data.insert(recovered_data.end(), 
                                working_blocks[i].begin(), 
                                working_blocks[i].end());
        }
        return true;
    }

    // Perform Reed-Solomon decoding
    std::cout << "[FEC] Attempting jerasure_matrix_decode with " << erasures.size()-1 << " erasures" << std::endl;
    std::cout << "[FEC] Erasures: ";
    for (int i = 0; i < erasures.size()-1; ++i) {
        std::cout << erasures[i] << " ";
    }
    std::cout << std::endl;
    
    std::cout << "[FEC] Block sizes: ";
    for (int i = 0; i < k_ + r_; ++i) {
        std::cout << working_blocks[i].size() << " ";
    }
    std::cout << std::endl;
    
    // Validate pointers before calling jerasure
    for (int i = 0; i < k_; ++i) {
        if (!data_ptrs[i]) {
            std::cerr << "[FEC] ERROR: data_ptrs[" << i << "] is null!" << std::endl;
            return false;
        }
    }
    for (int i = 0; i < r_; ++i) {
        if (!code_ptrs[i]) {
            std::cerr << "[FEC] ERROR: code_ptrs[" << i << "] is null!" << std::endl;
            return false;
        }
    }
    
    std::cout << "[FEC] All pointers validated, calling jerasure_matrix_decode..." << std::endl;
    
    int ret = jerasure_matrix_decode(k_, r_, w_, matrix_, 0, erasures.data(),
                                     reinterpret_cast<char**>(data_ptrs.data()),
                                     reinterpret_cast<char**>(code_ptrs.data()),
                                     block_size);
                                     
    std::cout << "[FEC] jerasure_matrix_decode returned: " << ret << std::endl;
    
    if (ret < 0) {
        std::cerr << "[FEC] Decode failed with error: " << ret << std::endl;
        return false;
    }

    // Extract recovered data
    recovered_data.clear();
    for (int i = 0; i < k_; ++i) {
        recovered_data.insert(recovered_data.end(), 
                            working_blocks[i].begin(), 
                            working_blocks[i].end());
    }
    
    return true;
}

