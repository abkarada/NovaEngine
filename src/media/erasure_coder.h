#pragma once

#include <vector>
#include <memory>

class ErasureCoder {
public:
    ErasureCoder();
    ~ErasureCoder();
    
    std::vector<std::vector<uint8_t>> encode(const std::vector<uint8_t>& data, int k, int n);
    std::vector<uint8_t> decode(const std::vector<std::vector<uint8_t>>& chunks, int k, int n);

private:
    // Implementation details
}; 