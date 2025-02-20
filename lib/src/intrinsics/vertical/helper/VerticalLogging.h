#pragma once
#include <accurate_ri.h>
#include <string>
#include "utils/Logger.h"

namespace accurate_ri::VerticalLogging {
    void printHeaderDebugInfo(const PointArray &points, const HoughTransform &hough) {
        LOG_INFO("==| Parameters |==");
        LOG_INFO("Number of points: ", points.size());
        LOG_INFO("Cloud path: ", getCloudPath()); // TODO remove this
        LOG_INFO("Coord distortion compensation strategy: ", "upper_bound");
        LOG_INFO("Hough model: ", "arcsin");
        LOG_INFO("Voting strategy: ", "single");
        LOG_INFO("Noise radius use square: ", "False");
        LOG_INFO("Fit type: ", "linear");
        LOG_INFO(
            "Offset min: ", hough.getXMin(), ", Offset max: ", hough.getXMax(), ", Offset res: ", hough.getXStep()
        );
        LOG_INFO("Angle min: ", hough.getYMin(), ", Angle max: ", hough.getYMax(), ", Angle res: ", hough.getYStep());
        LOG_INFO("Coords eps: ", points.getCoordsEps());
        LOG_INFO("");
        LOG_INFO("==| Execution |==");
    }
}
