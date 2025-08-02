#include "path_monitor.h"
#include <iostream>

PathMonitor::PathMonitor() {
    std::cout << "PathMonitor initialized" << std::endl;
}

PathMonitor::~PathMonitor() {
    std::cout << "PathMonitor destroyed" << std::endl;
}

void PathMonitor::addPath(const std::string& path) {
    std::cout << "Added path: " << path << std::endl;
}

void PathMonitor::removePath(const std::string& path) {
    std::cout << "Removed path: " << path << std::endl;
}

double PathMonitor::getRTT(const std::string& path) {
    // Return a dummy RTT value
    return 10.0;
}

double PathMonitor::getLossRate(const std::string& path) {
    // Return a dummy loss rate
    return 0.01;
} 