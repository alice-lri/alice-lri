#pragma once
#include <chrono>
#include <iostream>
#include <map>

#include "Logger.h"

#ifdef ENABLE_PROFILING
#define PROFILE_SCOPE(name) Timer timer##__LINE__ (name)
#define PRINT_PROFILE_REPORT() Timer::printSortedReport()
#else
#define PROFILE_SCOPE(name) do {} while (0)
#define PRINT_PROFILE_REPORT() do {} while (0)
#endif

#ifdef ENABLE_PROFILING

class Timer {
private:
    const char *name;
    std::chrono::high_resolution_clock::time_point start;

public:
    explicit Timer(const char *name) : name(name), start(std::chrono::high_resolution_clock::now()) {
    }

    ~Timer() {
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        getTotalTimeMap()[std::string(name)] += duration.count();
        LOG_DEBUG("[TIMER] ", std::string(name), " took ", duration.count(), " seconds");
    }

    static void printSortedReport() {
        std::vector<std::pair<std::string, double>> sortedTimes(getTotalTimeMap().begin(), getTotalTimeMap().end());
        std::sort(sortedTimes.begin(), sortedTimes.end(), [](const auto &a, const auto &b) {
            return a.second > b.second;
        });

        LOG_INFO("Profiling Report:");
        for (const auto &entry : sortedTimes) {
            LOG_INFO(entry.first, ": ", entry.second, " seconds");
        }
    }

private:
    static std::unordered_map<std::string, double>& getTotalTimeMap() {
        static std::unordered_map<std::string, double> totalTimeMap;
        return totalTimeMap;
    }
};
#endif