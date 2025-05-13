#include "VerticalLogging.h"
#include <accurate_ri/accurate_ri.hpp>
#include <nlohmann/json_fwd.hpp>

#include "../../../utils/json/JsonConverters.h"

namespace accurate_ri::VerticalLogging {

#if LOG_LEVEL <= LOG_LEVEL_INFO
    void printHeaderDebugInfo(const PointArray &points, const VerticalScanlinePool &hough) {
        LOG_INFO("==| Parameters |==");
        LOG_INFO("Number of points: ", points.size());
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
#else
    void printHeaderDebugInfo(const PointArray &points, const VerticalScanlinePool &hough) {}
#endif
} // namespace accurate_ri::VerticalLogging
