#pragma once
#include <iostream>
#include <fstream>
#include <mutex>
#include <ctime>
#include <sstream>

// ANSI Escape Codes for Colors
#define COLOR_RESET  "\033[0m"
#define COLOR_DEBUG  "\033[36m"  // Cyan
#define COLOR_INFO   "\033[32m"  // Green
#define COLOR_WARN   "\033[33m"  // Yellow
#define COLOR_ERROR  "\033[31m"  // Red

// Define logging levels
#define LOG_LEVEL_DEBUG 1
#define LOG_LEVEL_INFO  2
#define LOG_LEVEL_WARN  3
#define LOG_LEVEL_ERROR 4
#define LOG_LEVEL_NONE  5

enum LogLevel {
    Debug = LOG_LEVEL_DEBUG,
    Info = LOG_LEVEL_INFO,
    Warn = LOG_LEVEL_WARN,
    Error = LOG_LEVEL_ERROR,
    None = LOG_LEVEL_NONE
};

// Default log level (can be overridden at compile-time)
#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

class Logger {
public:
#if LOG_LEVEL <= LOG_LEVEL_ERROR
    // Core logging function using variadic template for `<<` support
    template <typename... Args>
    static void log(LogLevel level, const char* file, int line, Args&&... args) {
        std::ostringstream logStream;
        logStream << getTimestamp() << " [" << getLogLevelString(level) << "] ";
        (logStream << ... << std::forward<Args>(args));  // Fold expression for variadic logging
        logStream << " (" << file << ":" << line << ")\n";

        std::lock_guard<std::mutex> lock(getInstance().logMutex);
            std::cerr << getColor(level) << logStream.str() << COLOR_RESET;
        }
#endif

private:
#if LOG_LEVEL <= LOG_LEVEL_ERROR
    std::mutex logMutex;
#endif

    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

#if LOG_LEVEL <= LOG_LEVEL_ERROR
    static std::string getTimestamp() {
        std::time_t now = std::time(nullptr);
        std::tm tmNow;
        localtime_r(&now, &tmNow);
        char buf[20];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmNow);
        return std::string(buf);
    }

    static std::string getLogLevelString(LogLevel level) {
        switch (level) {
            case Debug: return "DEBUG";
            case Info:  return "INFO";
            case Warn:  return "WARN";
            case Error: return "ERROR";
            default: return "UNKNOWN";
        }
    }

    static const char* getColor(LogLevel level) {
        switch (level) {
            case Debug: return COLOR_DEBUG;
            case Info:  return COLOR_INFO;
            case Warn:  return COLOR_WARN;
            case Error: return COLOR_ERROR;
            default: return COLOR_RESET;
        }
    }
#endif
};

// **Compile-Time Optimized Logging Macros (Now with `<<` support!)**
#if LOG_LEVEL <= LOG_LEVEL_DEBUG
    #define LOG_DEBUG(...) Logger::log(Debug, __FILE__, __LINE__, __VA_ARGS__)
#else
    #define LOG_DEBUG(...) do {} while (0)
#endif

#if LOG_LEVEL <= LOG_LEVEL_INFO
    #define LOG_INFO(...) Logger::log(Info, __FILE__, __LINE__, __VA_ARGS__)
#else
    #define LOG_INFO(...) do {} while (0)
#endif

#if LOG_LEVEL <= LOG_LEVEL_WARN
    #define LOG_WARN(...) Logger::log(Warn, __FILE__, __LINE__, __VA_ARGS__)
#else
    #define LOG_WARN(...) do {} while (0)
#endif

#if LOG_LEVEL <= LOG_LEVEL_ERROR
    #define LOG_ERROR(...) Logger::log(Error, __FILE__, __LINE__, __VA_ARGS__)
#else
    #define LOG_ERROR(...) do {} while (0)
#endif
