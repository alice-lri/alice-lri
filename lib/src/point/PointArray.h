#pragma once

#include <cassert>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>

namespace accurate_ri {
    struct PointArrayExtraInfo {
        Eigen::VectorXd range;
        double coordsEps = 0;
    };

    class PointArray {
    private:
        std::vector<double> x, y, z;
        mutable PointArrayExtraInfo extraInfo;

    public:
        PointArray(std::vector<double> &&x_, std::vector<double> &&y_, std::vector<double> &&z_)
            : x(std::move(x_)), y(std::move(y_)), z(std::move(z_)) {
            computeExtraInfo();
        }

        PointArray(const std::vector<double> &x_, const std::vector<double> &y_, const std::vector<double> &z_)
            : x(x_), y(y_), z(z_) {
            computeExtraInfo();
        }

        [[nodiscard]] inline const std::vector<double>& getX() const { return x; }
        [[nodiscard]] inline const std::vector<double>& getY() const { return y; }
        [[nodiscard]] inline const std::vector<double>& getZ() const { return z; }

        [[nodiscard]] inline double getX(const size_t index) const { return x[index]; }
        [[nodiscard]] inline double getY(const size_t index) const { return y[index]; }
        [[nodiscard]] inline double getZ(const size_t index) const { return z[index]; }

        [[nodiscard]] inline double getRange(const size_t index) const { return extraInfo.range[index]; }
        [[nodiscard]] inline double getPhi(const size_t index) const { return extraInfo.phi[index]; }
        [[nodiscard]] inline double getTheta(const size_t index) const { return extraInfo.theta[index]; }
        [[nodiscard]] inline double getCoordsEps() const { return extraInfo.coordsEps; }

        [[nodiscard]] inline const std::vector<double>& getRanges() const { return extraInfo.range; }
        [[nodiscard]] inline const std::vector<double>& getPhis() const { return extraInfo.phi; }
        [[nodiscard]] inline const std::vector<double>& getThetas() const { return extraInfo.theta; }

        [[nodiscard]] size_t size() const { return x.size(); }

    private:
        void computeExtraInfo();
    };
}
