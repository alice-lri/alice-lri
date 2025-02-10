#pragma once
#include <chrono>
#include <iostream>
#include "Logger.h"

#ifdef ENABLE_PROFILING
#define PROFILE_SCOPE(name) Timer timer##__LINE__ (name)
#else
#define PROFILE_SCOPE(name) do {} while (0)
#endif


class Timer {
private:
    std::chrono::high_resolution_clock::time_point start;
    const char* name;

public:
    explicit Timer(const char* name) : name(name), start(std::chrono::high_resolution_clock::now()) {}

    ~Timer() {
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        LOG_INFO("[TIMER] " + std::string(name) + " took " + std::to_string(duration.count()) + " seconds");
    }
};
