#pragma once

#include <memory>
#include <vector>
#include <functional>

class Scheduler {
public:
    Scheduler();
    ~Scheduler();
    
    void schedule(std::function<void()> task);
    void start();
    void stop();

private:
    // Implementation details
}; 