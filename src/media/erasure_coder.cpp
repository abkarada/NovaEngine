#include "erasure_coder.h"
#include <iostream>

ErasureCoder::ErasureCoder() {
    std::cout << "ErasureCoder initialized" << std::endl;
}

ErasureCoder::~ErasureCoder() {
    std::cout << "ErasureCoder destroyed" << std::endl;
}

std::vector<std::vector<uint8_t>> ErasureCoder::encode(const std::vector<uint8_t>& data, int k, int n) {
    // Basic implementation - just split data into k chunks
    std::vector<std::vector<uint8_t>> chunks;
    size_t chunk_size = data.size() / k;
    
    for (int i = 0; i < k; ++i) {
        std::vector<uint8_t> chunk;
        size_t start = i * chunk_size;
        size_t end = (i == k - 1) ? data.size() : (i + 1) * chunk_size;
        chunk.assign(data.begin() + start, data.begin() + end);
        chunks.push_back(chunk);
    }
    
    // Add parity chunks (empty for now)
    for (int i = k; i < n; ++i) {
        chunks.push_back(std::vector<uint8_t>());
    }
    
    return chunks;
}

std::vector<uint8_t> ErasureCoder::decode(const std::vector<std::vector<uint8_t>>& chunks, int k, int n) {
    // Basic implementation - just concatenate first k chunks
    std::vector<uint8_t> result;
    for (int i = 0; i < k && i < chunks.size(); ++i) {
        result.insert(result.end(), chunks[i].begin(), chunks[i].end());
    }
    return result;
} 