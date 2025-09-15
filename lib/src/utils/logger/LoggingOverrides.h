#pragma once
#include <iomanip>
#include <ios>
#include <iosfwd>
#include <unordered_set>
#include <vector>
#include <Eigen/Core>
#include "alice_lri/public_structs.hpp"

template<typename T>
std::ostream &operator<<(std::ostream &os, const std::unordered_set<T> &set) {
    os << std::fixed << std::setprecision(5) << "{";

    bool firstElement = true;
    for (const auto &element: set) {
        if (!firstElement) {
            os << ", ";
        }
        os << element;
        firstElement = false;
    }

    os << "}";
    return os;
}

template<typename T>
std::ostream &operator<<(std::ostream &os, const Eigen::ArrayX<T> &array) {
    os << std::fixed << std::setprecision(5) << "[";

    bool firstElement = true;
    for (const auto &element: array) {
        if (!firstElement) {
            os << ", ";
        }
        os << element;
        firstElement = false;
    }

    os << "]";
    return os;
}

template<typename T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &vec) {
    os << std::fixed << std::setprecision(5) << "[";

    bool firstElement = true;
    for (const auto &element: vec) {
        if (!firstElement) {
            os << ", ";
        }
        os << element;
        firstElement = false;
    }

    os << "]";
    return os;
}

inline std::ostream &operator<<(std::ostream &os, const alice_lri::Interval &interval) {
    os << std::fixed << std::setprecision(5) << "[" << interval.lower << ", " << interval.upper << "]";
    return os;
}
