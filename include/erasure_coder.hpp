#pragma once

#include <vector>
#include <cstdint>
#include <memory>

// NovaEngine Ultra Stream - True Reed-Solomon FEC
// High-performance erasure coding with libjerasure

class ErasureCoder {
public:
    ErasureCoder(int k_chunks = 6, int r_chunks = 2, int chunk_size = 1000);
    ~ErasureCoder();
    
    // Encode data into k+r chunks with Reed-Solomon FEC
    std::vector<std::vector<uint8_t>> encode(const std::vector<uint8_t>& data);
    
    // Decode data from available chunks (minimum k required)
    std::vector<uint8_t> decode(const std::vector<std::vector<uint8_t>>& chunks,
                               const std::vector<bool>& chunk_available);
    
    // Check if frame can be decoded with available chunks
    bool canDecode(const std::vector<bool>& chunk_available) const;
    
    // Get coding statistics
    struct CodingStats {
        uint32_t frames_encoded;
        uint32_t frames_decoded;
        uint32_t frames_recovered;  // Decoded with FEC
        uint32_t frames_lost;       // Failed to decode
        double recovery_rate;       // Recovery success rate
        uint64_t total_encode_time_us;
        uint64_t total_decode_time_us;
    };
    
    CodingStats getStats() const;
    
    // Set FEC parameters
    void setParams(int k, int r, int chunk_size);
    
    // Get current parameters
    int getK() const { return k_; }
    int getR() const { return r_; }
    int getChunkSize() const { return chunk_size_; }
    int getTotalChunks() const { return k_ + r_; }

private:
    int k_;                      // Data chunks
    int r_;                      // Parity chunks
    int chunk_size_;             // Size of each chunk
    int* rs_matrix_;             // Reed-Solomon encoding matrix
    int* rs_bitmatrix_;          // Bit matrix for faster operations
    CodingStats stats_;          // Coding statistics
    
    // Initialize Reed-Solomon matrices
    bool initMatrices();
    
    // Clean up matrices
    void cleanupMatrices();
    
    // Pad data to fit k chunks exactly
    std::vector<uint8_t> padData(const std::vector<uint8_t>& data);
    
    // Remove padding from decoded data
    std::vector<uint8_t> unpadData(const std::vector<uint8_t>& data, size_t original_size);
    
    // Create erasures list for decoding
    std::vector<int> createErasuresList(const std::vector<bool>& chunk_available);
};

// Global erasure coder interface
extern ErasureCoder* g_erasure_coder;

void initErasureCoder(int k = 6, int r = 2, int chunk_size = 1000);
std::vector<std::vector<uint8_t>> encodeFrame(const std::vector<uint8_t>& data);
std::vector<uint8_t> decodeFrame(const std::vector<std::vector<uint8_t>>& chunks,
                                const std::vector<bool>& chunk_available);
void shutdownErasureCoder();

