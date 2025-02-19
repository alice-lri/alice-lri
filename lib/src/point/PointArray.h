#pragma once

#include <cassert>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>

namespace accurate_ri {
    struct PointArrayExtraInfo {
        Eigen::ArrayXd range, rangeXy, phi, theta = Eigen::ArrayXd::Zero(0);
        Eigen::ArrayXd invRange = Eigen::ArrayXd::Zero(0);
        double coordsEps = 0;
    };

    class PointArray {
    private:
        Eigen::ArrayXd x, y, z;
        mutable PointArrayExtraInfo extraInfo;

    public:
        PointArray(Eigen::ArrayXd &&x_, Eigen::ArrayXd &&y_, Eigen::ArrayXd &&z_)
            : x(std::move(x_)), y(std::move(y_)), z(std::move(z_)) {
            computeExtraInfo();
        }

        PointArray(const Eigen::ArrayXd &x_, const Eigen::ArrayXd &y_, const Eigen::ArrayXd &z_)
            : x(x_), y(y_), z(z_) {
            computeExtraInfo();
        }

        [[nodiscard]] inline const Eigen::ArrayXd& getX() const { return x; }
        [[nodiscard]] inline const Eigen::ArrayXd& getY() const { return y; }
        [[nodiscard]] inline const Eigen::ArrayXd& getZ() const { return z; }

        [[nodiscard]] inline double getX(const size_t index) const { return x[index]; }
        [[nodiscard]] inline double getY(const size_t index) const { return y[index]; }
        [[nodiscard]] inline double getZ(const size_t index) const { return z[index]; }

        [[nodiscard]] inline double getRange(const size_t index) const { return extraInfo.range[index]; }
        [[nodiscard]] inline double getRangeXy(const size_t index) const { return extraInfo.rangeXy[index]; }
        [[nodiscard]] inline double getPhi(const size_t index) const { return extraInfo.phi[index]; }
        [[nodiscard]] inline double getTheta(const size_t index) const { return extraInfo.theta[index]; }
        [[nodiscard]] inline double getCoordsEps() const { return extraInfo.coordsEps; }

        [[nodiscard]] inline double getInvRange(const size_t index) const { return extraInfo.invRange[index]; }

        [[nodiscard]] inline const Eigen::ArrayXd& getRanges() const { return extraInfo.range; }
        [[nodiscard]] inline const Eigen::ArrayXd& getRangesXy() const { return extraInfo.rangeXy; }
        [[nodiscard]] inline const Eigen::ArrayXd& getPhis() const { return extraInfo.phi; }
        [[nodiscard]] inline const Eigen::ArrayXd& getThetas() const { return extraInfo.theta; }

        [[nodiscard]] inline const Eigen::ArrayXd& getInvRanges() const { return extraInfo.invRange; }

        [[nodiscard]] size_t size() const { return x.size(); }

    private:
        void computeExtraInfo();
    };
}
