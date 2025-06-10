#pragma once
#include <iostream>
#include <fstream>
#include <mutex>
#include <ctime>
#include <sstream>
#include <iomanip>

#define COLOR_RESET  "\033[0m"
#define COLOR_DEBUG  "\033[36m"
#define COLOR_INFO   "\033[32m"
#define COLOR_WARN   "\033[33m"
#define COLOR_ERROR  "\033[31m"

#define LOG_LEVEL_DEBUG 1
#define LOG_LEVEL_INFO  2
#define LOG_LEVEL_WARN  3
#define LOG_LEVEL_ERROR 4
#define LOG_LEVEL_NONE  5

namespace accurate_ri {
    enum LogLevel {
        Debug = LOG_LEVEL_DEBUG,
        Info = LOG_LEVEL_INFO,
        Warn = LOG_LEVEL_WARN,
        Error = LOG_LEVEL_ERROR,
        None = LOG_LEVEL_NONE
    };

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

    class Logger {
    public:
#if LOG_LEVEL <= LOG_LEVEL_ERROR
        template<typename... Args>
        static void log(LogLevel level, const char *file, int line, Args &&... args) {
            std::ostringstream logStream;

            logStream << getTimestamp() << " [" << getLogLevelString(level) << "] ";
            (logStream << ... << std::forward<Args>(args));
            logStream << " (" << file << ":" << line << ")" << std::endl;

            std::lock_guard lock(getInstance().logMutex);
            std::cout << getColor(level) << logStream.str() << COLOR_RESET;
#ifdef ENABLE_TRACE_FILE
            std::ofstream traceFile("ref_tracing/trace.log", getInstance().firstLog ? std::ios_base::trunc : std::ios_base::app);
            traceFile << logStream.str();
            traceFile.close();
#endif
            getInstance().firstLog = false;
        }
#endif

    private:
#if LOG_LEVEL <= LOG_LEVEL_ERROR
        std::mutex logMutex;
#endif
        bool firstLog = true;

        static Logger &getInstance() {
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
                case Info: return "INFO";
                case Warn: return "WARN";
                case Error: return "ERROR";
                default: return "UNKNOWN";
            }
        }

        static const char *getColor(LogLevel level) {
            switch (level) {
                case Debug: return COLOR_DEBUG;
                case Info: return COLOR_INFO;
                case Warn: return COLOR_WARN;
                case Error: return COLOR_ERROR;
                default: return COLOR_RESET;
            }
        }
#endif
    };
}

    // **Compile-Time Optimized Logging Macros (Now with `<<` support!)**
#if LOG_LEVEL <= LOG_LEVEL_DEBUG
#define LOG_DEBUG(...) accurate_ri::Logger::log(accurate_ri::Debug, __FILE__, __LINE__, __VA_ARGS__)
#else
#define LOG_DEBUG(...) do {} while (0)
#endif

#if LOG_LEVEL <= LOG_LEVEL_INFO
#define LOG_INFO(...) accurate_ri::Logger::log(accurate_ri::Info, __FILE__, __LINE__, __VA_ARGS__)
#else
#define LOG_INFO(...) do {} while (0)
#endif

#if LOG_LEVEL <= LOG_LEVEL_WARN
#define LOG_WARN(...) accurate_ri::Logger::log(accurate_ri::Warn, __FILE__, __LINE__, __VA_ARGS__)
#else
#define LOG_WARN(...) do {} while (0)
#endif

#if LOG_LEVEL <= LOG_LEVEL_ERROR
#define LOG_ERROR(...) accurate_ri::Logger::log(accurate_ri::Error, __FILE__, __LINE__, __VA_ARGS__)
#else
#define LOG_ERROR(...) do {} while (0)
#endif
