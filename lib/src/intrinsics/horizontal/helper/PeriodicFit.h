#pragma once
#include "math/Stats.h"


namespace accurate_ri::PeriodicFit {
    Stats::LRResult fit(const Eigen::ArrayXd &x, const Eigen::ArrayXd &y, double period, double slopeGuess);
}
