#include "scheduler.h"
#include <iostream>

Scheduler::Scheduler() {
    std::cout << "Scheduler initialized" << std::endl;
}

Scheduler::~Scheduler() {
    std::cout << "Scheduler destroyed" << std::endl;
}

void Scheduler::schedule(std::function<void()> task) {
    // Basic implementation - just execute the task immediately
    if (task) {
        task();
    }
}

void Scheduler::start() {
    std::cout << "Scheduler started" << std::endl;
}

void Scheduler::stop() {
    std::cout << "Scheduler stopped" << std::endl;
} 