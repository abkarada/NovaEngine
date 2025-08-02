#include "chunk_dispatcher.hpp"
#include "scheduler.hpp"
#include "udp_sender.hpp"
#include <iostream>

void dispatch_chunk(const uint8_t* data, size_t size) {
    if (size < sizeof(ChunkPacket)) {
        std::cerr << "[dispatcher] Invalid packet size: " << size << std::endl;
        return;
    }
    
    const ChunkPacket& pkt = *reinterpret_cast<const ChunkPacket*>(data);
    
    try {
        // Use new scheduler function
        PathStats selected_path = selectPathForChunk(pkt.chunk_id);
        std::cout << "[dispatcher] Selected path: " << selected_path.ip << ":" << selected_path.port << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[dispatcher] Scheduler error: " << e.what() << std::endl;
    }
}
