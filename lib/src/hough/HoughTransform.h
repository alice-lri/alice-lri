#pragma once
#include <cstdint>
#include <vector>

namespace accurate_ri {

class HoughTransform {
private:
  std::vector<double> accumulator;
  std::vector<uint64_t> hashAccumulator;

  double xMin;
  double xMax;
  double xStep;
  double yMin;
  double yMax;
  double yStep;

  uint32_t xCount;
  uint32_t yCount;

public:
  HoughTransform(double xMin, double xMax, double xStep, double yMin, double yMax, double yStep);

  void computeAccumulator(const std::vector<double> &ranges, const std::vector<double> &phis);

private:
  inline void updateAccumulatorForPoint(uint64_t pointIndex, const std::vector<double> &ranges,
                                        const std::vector<double> &phis,
                                        std::vector<double> xValues);

  inline void voteForDiscontinuities(uint64_t pointIndex, int32_t previousY, size_t x, int32_t y, double voteVal);
};

} // accurate_ri
