#pragma once

#include <memory>
#include <vector>
#include <string>

class PathMonitor {
public:
    PathMonitor();
    ~PathMonitor();
    
    void addPath(const std::string& path);
    void removePath(const std::string& path);
    double getRTT(const std::string& path);
    double getLossRate(const std::string& path);

private:
    // Implementation details
}; 