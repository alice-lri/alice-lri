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
    static void setLogFile(const std::string& filename) {
#if LOG_LEVEL <= LOG_LEVEL_ERROR
        std::lock_guard<std::mutex> lock(getInstance().logMutex);
        getInstance().logFile.open(filename, std::ios::out | std::ios::app);
#endif
    }

#if LOG_LEVEL <= LOG_LEVEL_ERROR
    static void log(LogLevel level, const std::string& msg, const char* file, int line) {
        std::ostringstream logStream;
        logStream << getTimestamp() << " [" << getLogLevelString(level) << "] " << msg
                  << " (" << file << ":" << line << ")\n";

        std::lock_guard<std::mutex> lock(getInstance().logMutex);

        if (getInstance().logFile.is_open()) {
            getInstance().logFile << logStream.str();
            getInstance().logFile.flush();
        } else {
            std::cerr << getColor(level) << logStream.str() << COLOR_RESET;
        }
    }
#endif

private:
#if LOG_LEVEL <= LOG_LEVEL_ERROR
    std::ofstream logFile;
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

// **Compile-Time Optimized Logging Macros**
#if LOG_LEVEL <= LOG_LEVEL_DEBUG
    #define LOG_DEBUG(msg) Logger::log(Debug, msg, __FILE__, __LINE__)
#else
    #define LOG_DEBUG(msg) do {} while (0)
#endif

#if LOG_LEVEL <= LOG_LEVEL_INFO
    #define LOG_INFO(msg) Logger::log(Info, msg, __FILE__, __LINE__)
#else
    #define LOG_INFO(msg) do {} while (0)
#endif

#if LOG_LEVEL <= LOG_LEVEL_WARN
    #define LOG_WARN(msg) Logger::log(Warn, msg, __FILE__, __LINE__)
#else
    #define LOG_WARN(msg) do {} while (0)
#endif

#if LOG_LEVEL <= LOG_LEVEL_ERROR
    #define LOG_ERROR(msg) Logger::log(Error, msg, __FILE__, __LINE__)
#else
    #define LOG_ERROR(msg) do {} while (0)
#endif
