// slicer.h
#pragma once

#include <vector>
#include <cstdint>
#include <memory>
#include <cstring>
#include <algorithm>

// MTU-safe payload size
constexpr size_t PAYLOAD_SIZE = 1000;

// Header for each UDP chunk
struct ChunkHeader {
    uint32_t frame_id;      // Identifies the frame
    uint16_t chunk_id;      // 0 to k-1 are data, k to k+r-1 are parity
    uint16_t k;             // Number of data chunks
    uint16_t r;             // Number of parity chunks
    uint16_t chunk_size;    // Size of this chunk's data
};

struct Chunk {
    ChunkHeader header;
    std::vector<uint8_t> payload;

    // Serialize the chunk for sending over UDP
    std::vector<uint8_t> serialize() const {
        std::vector<uint8_t> buffer(sizeof(ChunkHeader) + payload.size());
        memcpy(buffer.data(), &header, sizeof(ChunkHeader));
        memcpy(buffer.data() + sizeof(ChunkHeader), payload.data(), payload.size());
        return buffer;
    }

    // Deserialize a chunk from UDP data
    static Chunk deserialize(const std::vector<uint8_t>& buffer) {
        Chunk chunk;
        if (buffer.size() < sizeof(ChunkHeader)) return chunk; // Invalid
        memcpy(&chunk.header, buffer.data(), sizeof(ChunkHeader));
        if (buffer.size() >= sizeof(ChunkHeader) + chunk.header.chunk_size) {
             chunk.payload.assign(buffer.begin() + sizeof(ChunkHeader), buffer.begin() + sizeof(ChunkHeader) + chunk.header.chunk_size);
        }
        return chunk;
    }
};

using ChunkPtr = std::shared_ptr<Chunk>;

class Slicer {
public:
    static std::vector<ChunkPtr> slice(const std::vector<uint8_t>& frame_data, uint32_t frame_id, uint16_t k, uint16_t r) {
        std::vector<ChunkPtr> chunks;
        size_t total_size = frame_data.size();
        size_t offset = 0;

        for (uint16_t i = 0; i < k; ++i) {
            auto chunk = std::make_shared<Chunk>();
            size_t current_chunk_size = std::min(PAYLOAD_SIZE, total_size - offset);
            
            chunk->header.frame_id = frame_id;
            chunk->header.chunk_id = i;
            chunk->header.k = k;
            chunk->header.r = r;
            chunk->header.chunk_size = static_cast<uint16_t>(current_chunk_size);
            
            chunk->payload.resize(PAYLOAD_SIZE, 0); // All data chunks must be same size for FEC
            if (current_chunk_size > 0) {
                 memcpy(chunk->payload.data(), frame_data.data() + offset, current_chunk_size);
            }

            chunks.push_back(std::move(chunk));
            offset += current_chunk_size;
            if(offset >= total_size) break;
        }
        // Pad with empty chunks if necessary
        while(chunks.size() < k) {
            auto chunk = std::make_shared<Chunk>();
            chunk->header = chunks[0]->header; // Copy header from first chunk
            chunk->header.chunk_id = chunks.size();
            chunk->payload.resize(PAYLOAD_SIZE, 0);
            chunks.push_back(chunk);
        }

        return chunks;
    }
};