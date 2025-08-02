#include "slicer.hpp"
#include "erasure_coder.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <cstring>

// NovaEngine Ultra Stream - MTU-Aware Frame Slicer
// Intelligent frame chunking with Reed-Solomon FEC preparation

// Global slicer instance
FrameSlicer* g_slicer = nullptr;

FrameSlicer::FrameSlicer(int chunk_size, int k_chunks, int r_chunks)
    : chunk_size_(chunk_size), k_chunks_(k_chunks), r_chunks_(r_chunks) {
    std::cout << "[Slicer] Initialized with chunk_size=" << chunk_size_ 
              << ", k=" << k_chunks_ << ", r=" << r_chunks_ << std::endl;
}

FrameSlicer::~FrameSlicer() {
}

std::vector<ChunkPacket> FrameSlicer::sliceFrame(const std::vector<uint8_t>& frame_data, uint32_t frame_id) {
    if (frame_data.empty()) {
        return {};
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Use Reed-Solomon FEC encoding
    std::vector<std::vector<uint8_t>> fec_chunks;
    if (!g_erasure_coder || !g_erasure_coder->encode(frame_data, fec_chunks)) {
        std::cerr << "[Slicer] FEC encoding failed" << std::endl;
        return {};
    }
    
    std::vector<ChunkPacket> packets;
    packets.reserve(fec_chunks.size());
    
    // Create packets for each FEC chunk
    for (size_t i = 0; i < fec_chunks.size(); ++i) {
        ChunkPacket packet;
        packet.frame_id = frame_id;
        packet.chunk_id = static_cast<uint16_t>(i);
        packet.total_chunks = static_cast<uint16_t>(fec_chunks.size());
        packet.is_parity = (i >= k_chunks_);  // Mark parity chunks
        packet.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        packet.path_id = 0;
        packet.priority = (i < k_chunks_) ? 1 : 0;  // Data chunks have higher priority
        
        // Copy data to packet
        packet.data_size = std::min(static_cast<uint16_t>(fec_chunks[i].size()), 
                                   static_cast<uint16_t>(sizeof(packet.data)));
        std::memcpy(packet.data, fec_chunks[i].data(), packet.data_size);
        
        packets.push_back(packet);
    }
    
    // Update statistics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    std::cout << "[Slicer] Frame " << frame_id << " sliced into " << packets.size() 
              << " chunks (" << k_chunks_ << " data + " << r_chunks_ << " parity) in " 
              << duration.count() << "μs" << std::endl;
    
    return packets;
}

std::vector<uint8_t> FrameSlicer::reconstructFrame(const std::vector<ChunkPacket>& chunks) {
    if (chunks.empty()) return {};
    
    uint32_t frame_id = chunks[0].frame_id;
    std::vector<uint8_t> reconstructed_data;
    
    // Sort chunks by chunk_id
    std::vector<ChunkPacket> sorted_chunks = chunks;
    std::sort(sorted_chunks.begin(), sorted_chunks.end(), 
              [](const ChunkPacket& a, const ChunkPacket& b) {
                  return a.chunk_id < b.chunk_id;
              });
    
    // Reconstruct from data chunks only
    for (const auto& chunk : sorted_chunks) {
        if (chunk.chunk_id < k_chunks_) {
            reconstructed_data.insert(reconstructed_data.end(), 
                                     chunk.data, 
                                     chunk.data + chunk.data_size);
        }
    }
    
    std::cout << "[Slicer] Frame " << frame_id << " reconstructed: " 
              << reconstructed_data.size() << " bytes" << std::endl;
    
    return reconstructed_data;
}

FrameSlicer::SliceStats FrameSlicer::getLastSliceStats() const {
    return last_stats_;
}

void FrameSlicer::setChunkSize(int size) {
    chunk_size_ = size;
    std::cout << "[Slicer] Chunk size updated to " << chunk_size_ << " bytes" << std::endl;
}

void FrameSlicer::setFECParams(int k, int r) {
    k_chunks_ = k;
    r_chunks_ = r;
    std::cout << "[Slicer] FEC parameters updated: k=" << k_chunks_ << ", r=" << r_chunks_ << std::endl;
}

ChunkPacket FrameSlicer::createChunkPacket(uint32_t frame_id, uint16_t chunk_id, 
                                          uint16_t total_chunks, const std::vector<uint8_t>& data,
                                          uint64_t timestamp) {
    ChunkPacket packet{};
    packet.frame_id = frame_id;
    packet.chunk_id = chunk_id;
    packet.total_chunks = total_chunks;
    packet.timestamp = timestamp;
    packet.path_id = 0;
    packet.priority = (chunk_id < k_chunks_) ? 1 : 0;  // Data chunks have higher priority
    
    // Copy data to packet
    packet.data_size = std::min(static_cast<uint16_t>(data.size()), 
                               static_cast<uint16_t>(sizeof(packet.data)));
    std::memcpy(packet.data, data.data(), packet.data_size);
    
    return packet;
}

std::vector<uint8_t> FrameSlicer::padChunk(const std::vector<uint8_t>& data, int target_size) {
    std::vector<uint8_t> padded = data;
    
    if (padded.size() < static_cast<size_t>(target_size)) {
        // Pad with zeros to reach target size
        padded.resize(target_size, 0);
    } else if (padded.size() > static_cast<size_t>(target_size)) {
        // Truncate if too large
        padded.resize(target_size);
    }
    
    return padded;
}

// Global interface functions
void initSlicer(int chunk_size, int k, int r) {
    if (g_slicer) {
        delete g_slicer;
    }
    
    g_slicer = new FrameSlicer(chunk_size, k, r);
    std::cout << "[Slicer] ✅ Global slicer initialized" << std::endl;
}

std::vector<ChunkPacket> sliceFrame(const std::vector<uint8_t>& frame_data, 
                                   uint32_t frame_id, uint64_t timestamp) {
    if (!g_slicer) {
        throw std::runtime_error("Slicer not initialized");
    }
    
    return g_slicer->sliceFrame(frame_data, frame_id);
}

void shutdownSlicer() {
    if (g_slicer) {
        delete g_slicer;
        g_slicer = nullptr;
    }
    
    std::cout << "[Slicer] Slicer shutdown complete" << std::endl;
}

