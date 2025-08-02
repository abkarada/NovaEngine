#include "logger.h"

// Static member initialization
std::mutex Logger::log_mutex_;
LogLevel Logger::current_level_ = LogLevel::INFO;
Logger logger; 