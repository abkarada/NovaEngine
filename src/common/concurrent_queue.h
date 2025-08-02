#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>

template<typename T>
class ConcurrentQueue {
public:
    ConcurrentQueue() = default;
    ~ConcurrentQueue() = default;
    
    void push(const T& item);
    bool pop(T& item);
    bool empty() const;
    size_t size() const;

private:
    mutable std::mutex mutex_;
    std::queue<T> queue_;
    std::condition_variable condition_;
};

template<typename T>
void ConcurrentQueue<T>::push(const T& item) {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(item);
    condition_.notify_one();
}

template<typename T>
bool ConcurrentQueue<T>::pop(T& item) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (queue_.empty()) {
        return false;
    }
    item = queue_.front();
    queue_.pop();
    return true;
}

template<typename T>
bool ConcurrentQueue<T>::empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
}

template<typename T>
size_t ConcurrentQueue<T>::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
} 