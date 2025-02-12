#pragma once

#include <cassert>
#include <vector>
#include <cmath>

enum class PointArrayLayout { Coordinate, Point };

struct PointArrayExtraInfo {
    double coordsEps = 0;
};

template<PointArrayLayout L>
class PointArray;

template<>
class PointArray<PointArrayLayout::Coordinate> {
private:
    std::vector<double> x, y, z;
    std::vector<double> range, phi, theta;
    PointArrayExtraInfo extraInfo;

public:
    PointArray(std::vector<double> &&x_, std::vector<double> &&y_, std::vector<double> &&z_)
        : x(std::move(x_)), y(std::move(y_)), z(std::move(z_)),
          range(x.size()), phi(x.size()), theta(x.size()) {
        for (size_t i = 0; i < x.size(); ++i) {
            range[i] = std::sqrt(x[i] * x[i] + y[i] * y[i] + z[i] * z[i]);
            phi[i] = std::asin(z[i] / range[i]);
            theta[i] = std::atan2(y[i], x[i]);
        }
    }

    PointArray(const std::vector<double> &x_, const std::vector<double> &y_, const std::vector<double> &z_)
        : x(x_), y(y_), z(z_), range(x.size()), phi(x.size()), theta(x.size()) {
        for (size_t i = 0; i < x.size(); ++i) {
            range[i] = std::sqrt(x[i] * x[i] + y[i] * y[i] + z[i] * z[i]);
            phi[i] = std::asin(z[i] / range[i]);
            theta[i] = std::atan2(y[i], x[i]);
        }
    }

    [[nodiscard]] inline const std::vector<double>& getX() const { return x; }
    [[nodiscard]] inline const std::vector<double>& getY() const { return y; }
    [[nodiscard]] inline const std::vector<double>& getZ() const { return z; }

    [[nodiscard]] inline double getX(const size_t index) const { return x[index]; }
    [[nodiscard]] inline double getY(const size_t index) const { return y[index]; }
    [[nodiscard]] inline double getZ(const size_t index) const { return z[index]; }

    [[nodiscard]] inline double getRange(const size_t index) const { return range[index]; }
    [[nodiscard]] inline double getPhi(const size_t index) const { return phi[index]; }
    [[nodiscard]] inline double getTheta(const size_t index) const { return theta[index]; }

    [[nodiscard]] size_t size() const { return x.size(); }
};

template<>
class PointArray<PointArrayLayout::Point> {
private:
    struct Point {
        double x, y, z;
        double range, phi, theta;
    };

    std::vector<Point> points;

public:
    PointArray(const std::vector<double> &x_, const std::vector<double> &y_, const std::vector<double> &z_) {
        points.reserve(x_.size());
        for (size_t i = 0; i < x_.size(); ++i) {
            const double range = std::sqrt(x_[i] * x_[i] + y_[i] * y_[i] + z_[i] * z_[i]);
            const double phi = std::asin(z_[i] / range);
            const double theta = std::atan2(y_[i], x_[i]);

            points.push_back({x_[i], y_[i], z_[i], range, phi, theta});
        }
    }

    [[nodiscard]] inline const std::vector<double>& getX() const { return std::move(std::vector<double>()); }
    [[nodiscard]] inline const std::vector<double>& getY() const { return y; }
    [[nodiscard]] inline const std::vector<double>& getZ() const { return z; }

    [[nodiscard]] inline double getX(const size_t index) const { return points[index].x; }
    [[nodiscard]] inline double getY(const size_t index) const { return points[index].y; }
    [[nodiscard]] inline double getZ(const size_t index) const { return points[index].z; }

    [[nodiscard]] inline double getRange(const size_t index) const { return points[index].range; }
    [[nodiscard]] inline double getPhi(const size_t index) const { return points[index].phi; }
    [[nodiscard]] inline double getTheta(const size_t index) const { return points[index].theta; }

    [[nodiscard]] size_t size() const { return points.size(); }
};
