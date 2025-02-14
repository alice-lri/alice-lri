#pragma once

#include <cassert>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>

namespace accurate_ri {
    struct PointArrayExtraInfo {
        Eigen::VectorXd range, rangeXy, phi, theta = Eigen::VectorXd::Zero(0);
        Eigen::VectorXd invRange = Eigen::VectorXd::Zero(0);
        double coordsEps = 0;
    };

    class PointArray {
    private:
        Eigen::VectorXd x, y, z;
        mutable PointArrayExtraInfo extraInfo;

    public:
        PointArray(Eigen::VectorXd &&x_, Eigen::VectorXd &&y_, Eigen::VectorXd &&z_)
            : x(std::move(x_)), y(std::move(y_)), z(std::move(z_)) {
            computeExtraInfo();
        }

        PointArray(const Eigen::VectorXd &x_, const Eigen::VectorXd &y_, const Eigen::VectorXd &z_)
            : x(x_), y(y_), z(z_) {
            computeExtraInfo();
        }

        [[nodiscard]] inline const Eigen::VectorXd& getX() const { return x; }
        [[nodiscard]] inline const Eigen::VectorXd& getY() const { return y; }
        [[nodiscard]] inline const Eigen::VectorXd& getZ() const { return z; }

        [[nodiscard]] inline double getX(const size_t index) const { return x[index]; }
        [[nodiscard]] inline double getY(const size_t index) const { return y[index]; }
        [[nodiscard]] inline double getZ(const size_t index) const { return z[index]; }

        [[nodiscard]] inline double getRange(const size_t index) const { return extraInfo.range[index]; }
        [[nodiscard]] inline double getRangeXy(const size_t index) const { return extraInfo.rangeXy[index]; }
        [[nodiscard]] inline double getPhi(const size_t index) const { return extraInfo.phi[index]; }
        [[nodiscard]] inline double getTheta(const size_t index) const { return extraInfo.theta[index]; }
        [[nodiscard]] inline double getCoordsEps() const { return extraInfo.coordsEps; }

        [[nodiscard]] inline double getInvRange(const size_t index) const { return extraInfo.invRange[index]; }

        [[nodiscard]] inline const Eigen::VectorXd& getRanges() const { return extraInfo.range; }
        [[nodiscard]] inline const Eigen::VectorXd& getRangesXy() const { return extraInfo.rangeXy; }
        [[nodiscard]] inline const Eigen::VectorXd& getPhis() const { return extraInfo.phi; }
        [[nodiscard]] inline const Eigen::VectorXd& getThetas() const { return extraInfo.theta; }

        [[nodiscard]] inline const Eigen::VectorXd& getInvRanges() const { return extraInfo.invRange; }

        [[nodiscard]] size_t size() const { return x.size(); }

    private:
        void computeExtraInfo();
    };
}
