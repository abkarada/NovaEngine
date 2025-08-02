#pragma once

#include "udp_sender.hpp"
#include <vector>
#include <cstdint>

// NovaEngine Ultra Stream - MTU-Aware Frame Slicer
// Intelligent frame chunking with Reed-Solomon FEC preparation

class FrameSlicer {
public:
    FrameSlicer(int chunk_size = 1000, int k_chunks = 6, int r_chunks = 2);
    ~FrameSlicer();
    
    // Slice frame into chunks with Reed-Solomon FEC
    std::vector<ChunkPacket> sliceFrame(const std::vector<uint8_t>& frame_data, uint32_t frame_id);
    
    // Reconstruct frame from chunks (after FEC decode)
    std::vector<uint8_t> reconstructFrame(const std::vector<ChunkPacket>& chunks);
    
    // Get chunk statistics
    struct SliceStats {
        uint32_t frame_id;
        int total_chunks;
        int data_chunks;
        int parity_chunks;
        size_t original_size;
        size_t chunked_size;
        uint64_t slice_time_us;
    };
    
    SliceStats getLastSliceStats() const;
    
    // Set chunking parameters
    void setChunkSize(int size);
    void setFECParams(int k, int r);
    
    // Get current parameters
    int getChunkSize() const { return chunk_size_; }
    int getKChunks() const { return k_chunks_; }
    int getRChunks() const { return r_chunks_; }
    int getTotalChunks() const { return k_chunks_ + r_chunks_; }

private:
    int chunk_size_;           // MTU-optimized chunk size (default: 1000 bytes)
    int k_chunks_;            // Data chunks
    int r_chunks_;            // Parity chunks for FEC
    SliceStats last_stats_;   // Last slicing statistics
    
    // Create chunk packet with proper header
    ChunkPacket createChunkPacket(uint32_t frame_id, uint16_t chunk_id, 
                                 uint16_t total_chunks, const std::vector<uint8_t>& data,
                                 uint64_t timestamp);
    
    // Pad data to chunk size if necessary
    std::vector<uint8_t> padChunk(const std::vector<uint8_t>& data, int target_size);
};

// Global slicer interface
extern FrameSlicer* g_slicer;

void initSlicer(int chunk_size = 1000, int k = 6, int r = 2);
std::vector<ChunkPacket> sliceFrame(const std::vector<uint8_t>& frame_data, 
                                   uint32_t frame_id, uint64_t timestamp);
void shutdownSlicer();

