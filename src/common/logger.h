#pragma once

#include <iostream>
#include <string>
#include <sstream>

// Logging macros
#define LOG_ERROR(...) Logger::log(LogLevel::ERROR, __VA_ARGS__)
#define LOG_WARN(...)  Logger::log(LogLevel::WARN, __VA_ARGS__)
#define LOG_INFO(...)  Logger::log(LogLevel::INFO, __VA_ARGS__)
#define LOG_DEBUG(...) Logger::log(LogLevel::DEBUG, __VA_ARGS__)

enum class LogLevel {
    ERROR,
    WARN,
    INFO,
    DEBUG
};

class Logger {
public:
    template<typename... Args>
    static void log(LogLevel level, Args... args) {
        std::ostringstream oss;
        (oss << ... << args);
        std::cout << "[" << getLevelString(level) << "] " << oss.str() << std::endl;
    }

private:
    static const char* getLevelString(LogLevel level) {
        switch (level) {
            case LogLevel::ERROR: return "ERROR";
            case LogLevel::WARN:  return "WARN";
            case LogLevel::INFO:  return "INFO";
            case LogLevel::DEBUG: return "DEBUG";
            default: return "UNKNOWN";
        }
    }
}; 