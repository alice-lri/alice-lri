#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <string>
#include <vector>
#include <accurate_ri/public_structs.hpp>
#include <accurate_ri/accurate_ri.hpp>
#include <accurate_ri/Result.h>
#include <accurate_ri/AliceString.h>
#include <sstream>
#include <iostream>

namespace py = pybind11;

PYBIND11_MODULE(_accurate_ri, m) {
    m.doc() = "Python bindings for the AccurateRI C++ library";

    // Error handling
    py::enum_<accurate_ri::ErrorCode>(m, "ErrorCode")
        .value("NONE", accurate_ri::ErrorCode::NONE)
        .value("MISMATCHED_SIZES", accurate_ri::ErrorCode::MISMATCHED_SIZES)
        .value("EMPTY_POINT_CLOUD", accurate_ri::ErrorCode::EMPTY_POINT_CLOUD)
        .value("RANGES_XY_ZERO", accurate_ri::ErrorCode::RANGES_XY_ZERO)
        .value("INTERNAL_ERROR", accurate_ri::ErrorCode::INTERNAL_ERROR)
        .export_values();

    // Helper function to unwrap Result<T> and throw exceptions
    auto unwrap_result = [](const auto& result) -> auto {
        if (!result.ok()) {
            throw std::runtime_error(std::string(result.status().message.c_str()));
        }
        return result.value();
    };

    // Enums
    py::enum_<accurate_ri::EndReason>(m, "EndReason")
        .value("ALL_ASSIGNED", accurate_ri::EndReason::ALL_ASSIGNED)
        .value("MAX_ITERATIONS", accurate_ri::EndReason::MAX_ITERATIONS)
        .value("NO_MORE_PEAKS", accurate_ri::EndReason::NO_MORE_PEAKS)
        .export_values();

    // Core structs
    py::class_<accurate_ri::Scanline>(m, "Scanline")
        .def(py::init<>())
        .def_readwrite("vertical_offset", &accurate_ri::Scanline::verticalOffset)
        .def_readwrite("vertical_angle", &accurate_ri::Scanline::verticalAngle)
        .def_readwrite("horizontal_offset", &accurate_ri::Scanline::horizontalOffset)
        .def_readwrite("azimuthal_offset", &accurate_ri::Scanline::azimuthalOffset)
        .def_readwrite("resolution", &accurate_ri::Scanline::resolution)
        .def("__repr__", [](const accurate_ri::Scanline& self) {
            std::ostringstream oss;
            oss << "Scanline(vertical_offset=" << self.verticalOffset
                << ", vertical_angle=" << self.verticalAngle
                << ", horizontal_offset=" << self.horizontalOffset
                << ", azimuthal_offset=" << self.azimuthalOffset
                << ", resolution=" << self.resolution << ")";
            return oss.str();
        });

    py::class_<accurate_ri::Intrinsics>(m, "Intrinsics")
        .def(py::init<int32_t>())
        .def_property_readonly("scanlines", [](const accurate_ri::Intrinsics& self) {
            return std::vector(self.scanlines.begin(), self.scanlines.end());
        })
        .def("__repr__", [](const accurate_ri::Intrinsics& self) {
            std::ostringstream oss;
            oss << "Intrinsics(scanlines=[...])";
            return oss.str();
        });

    py::class_<accurate_ri::Interval>(m, "Interval")
        .def(py::init<>())
        .def_readwrite("lower", &accurate_ri::Interval::lower)
        .def_readwrite("upper", &accurate_ri::Interval::upper)
        .def("diff", &accurate_ri::Interval::diff)
        .def("any_contained", &accurate_ri::Interval::anyContained)
        .def("clamp_both", &accurate_ri::Interval::clampBoth)
        .def("__repr__", [](const accurate_ri::Interval& self) {
            std::ostringstream oss;
            oss << "Interval(lower=" << self.lower << ", upper=" << self.upper << ")";
            return oss.str();
        });

    py::class_<accurate_ri::ValueConfInterval>(m, "ValueConfInterval")
        .def(py::init<>())
        .def_readwrite("value", &accurate_ri::ValueConfInterval::value)
        .def_readwrite("ci", &accurate_ri::ValueConfInterval::ci)
        .def("__repr__", [](const accurate_ri::ValueConfInterval& self) {
            std::ostringstream oss;
            oss << "ValueConfInterval(value=" << self.value << ", ci=Interval(lower=" << self.ci.lower << ", upper=" << self.ci.upper << "))";
            return oss.str();
        });

    py::class_<accurate_ri::ScanlineAngleBounds>(m, "ScanlineAngleBounds")
        .def(py::init<>())
        .def_readwrite("lower_line", &accurate_ri::ScanlineAngleBounds::lowerLine)
        .def_readwrite("upper_line", &accurate_ri::ScanlineAngleBounds::upperLine)
        .def("__repr__", [](const accurate_ri::ScanlineAngleBounds& self) {
            std::ostringstream oss;
            oss << "ScanlineAngleBounds(" <<
                "lower_line=[" << self.lowerLine.lower << ", " << self.lowerLine.upper << "], " <<
                "upper_line=[" << self.upperLine.lower << ", " << self.upperLine.upper << "])";
            return oss.str();
        });

    py::class_<accurate_ri::DebugScanline>(m, "DebugScanline")
        .def(py::init<>())
        .def_readwrite("vertical_offset", &accurate_ri::DebugScanline::verticalOffset)
        .def_readwrite("vertical_angle", &accurate_ri::DebugScanline::verticalAngle)
        .def_readwrite("horizontal_offset", &accurate_ri::DebugScanline::horizontalOffset)
        .def_readwrite("azimuthal_offset", &accurate_ri::DebugScanline::azimuthalOffset)
        .def_readwrite("resolution", &accurate_ri::DebugScanline::resolution)
        .def_readwrite("uncertainty", &accurate_ri::DebugScanline::uncertainty)
        .def_readwrite("hough_votes", &accurate_ri::DebugScanline::houghVotes)
        .def_readwrite("hough_hash", &accurate_ri::DebugScanline::houghHash)
        .def_readwrite("points_count", &accurate_ri::DebugScanline::pointsCount)
        .def_readwrite("theoretical_angle_bounds", &accurate_ri::DebugScanline::theoreticalAngleBounds)
        .def_readwrite("vertical_heuristic", &accurate_ri::DebugScanline::verticalHeuristic)
        .def_readwrite("horizontal_heuristic", &accurate_ri::DebugScanline::horizontalHeuristic)
        .def("__repr__", [](const accurate_ri::DebugScanline& self) {
            std::ostringstream oss;
            oss << "DebugScanline(vertical_offset=" << self.verticalOffset.value
                << ", vertical_angle=" << self.verticalAngle.value
                << ", horizontal_offset=" << self.horizontalOffset
                << ", azimuthal_offset=" << self.azimuthalOffset
                << ", resolution=" << self.resolution
                << ", uncertainty=" << self.uncertainty
                << ", hough_votes=" << self.houghVotes
                << ", hough_hash=" << self.houghHash
                << ", points_count=" << self.pointsCount
                << ", theoretical_angle_bounds=ScanlineAngleBounds()"
                << ", vertical_heuristic=" << self.verticalHeuristic
                << ", horizontal_heuristic=" << self.horizontalHeuristic << ")";
            return oss.str();
        });

    py::class_<accurate_ri::DebugIntrinsics>(m, "DebugIntrinsics")
        .def(py::init<int32_t>())
        .def(py::init<int32_t, int32_t, int32_t, int32_t, accurate_ri::EndReason>())
        .def_property_readonly("scanlines", [](const accurate_ri::DebugIntrinsics& self) {
            return std::vector(self.scanlines.begin(), self.scanlines.end());
        })
        .def_readwrite("vertical_iterations", &accurate_ri::DebugIntrinsics::verticalIterations)
        .def_readwrite("unassigned_points", &accurate_ri::DebugIntrinsics::unassignedPoints)
        .def_readwrite("points_count", &accurate_ri::DebugIntrinsics::pointsCount)
        .def_readwrite("end_reason", &accurate_ri::DebugIntrinsics::endReason)
        .def("__repr__", [](const accurate_ri::DebugIntrinsics& self) {
            std::ostringstream oss;
            oss << "DebugIntrinsics(scanlines=[" << self.scanlines.size() << "], vertical_iterations=" << self.verticalIterations
                << ", unassigned_points=" << self.unassignedPoints
                << ", points_count=" << self.pointsCount
                << ", end_reason=" << static_cast<int>(self.endReason) << ")";
            return oss.str();
        });

    // RangeImage class
    py::class_<accurate_ri::RangeImage>(m, "RangeImage")
        .def(py::init<>())
        .def(py::init<uint32_t, uint32_t>(), py::arg("width"), py::arg("height"))
        .def(py::init<uint32_t, uint32_t, double>(), py::arg("width"), py::arg("height"), py::arg("initial_value"))
        .def_property_readonly("width", &accurate_ri::RangeImage::width)
        .def_property_readonly("height", &accurate_ri::RangeImage::height)
        .def("__repr__", [](const accurate_ri::RangeImage& self) {
            std::ostringstream oss;
            oss << "RangeImage(width=" << self.width() << ", height=" << self.height() << ")";
            return oss.str();
        })
        .def("__getitem__", [](const accurate_ri::RangeImage &ri, py::tuple idx) -> double {
            if (idx.size() != 2)
                throw py::index_error("Need 2 indices");
            size_t row = idx[0].cast<size_t>();
            size_t col = idx[1].cast<size_t>();
            if (row >= ri.height() || col >= ri.width())
                throw py::index_error("Index out of bounds");
            return ri(row, col);
        }, py::is_operator())
        .def("__setitem__", [](accurate_ri::RangeImage &ri, py::tuple idx, double value) {
            if (idx.size() != 2)
                throw py::index_error("Need 2 indices");
            size_t row = idx[0].cast<size_t>();
            size_t col = idx[1].cast<size_t>();
            if (row >= ri.height() || col >= ri.width())
                throw py::index_error("Index out of bounds");
            ri(row, col) = value;
        }, py::is_operator())
        .def("__array__", [](py::object self) {
            auto& ri = self.cast<const accurate_ri::RangeImage&>();
            return py::array_t<double>(
                {ri.height(), ri.width()},                        // shape
                {sizeof(double) * ri.width(), sizeof(double)},    // C-order strides
                ri.data(),                                         // pointer to data
                self                                              // keep alive
            );
        })
        .def("__repr__", [](const accurate_ri::RangeImage& self) {
            std::ostringstream oss;
            oss << "RangeImage(width=" << self.width() << ", height=" << self.height() << ")";
            return oss.str();
        });

    // Main API functions with vector inputs for convenience
    m.def("train", [&unwrap_result](const std::vector<float>& x, const std::vector<float>& y, const std::vector<float>& z) {
        // Convert std::vector to AliceArray
        accurate_ri::PointCloud::Float cloud;
        cloud.x = accurate_ri::AliceArray<float>(x.data(), x.size());
        cloud.y = accurate_ri::AliceArray<float>(y.data(), y.size());
        cloud.z = accurate_ri::AliceArray<float>(z.data(), z.size());
        return unwrap_result(accurate_ri::train(cloud));
    }, py::arg("x"), py::arg("y"), py::arg("z"), 
       "Estimate intrinsics from float vectors\n\n"
       "Parameters:\n"
       "  x: List of x coordinates\n" 
       "  y: List of y coordinates\n"
       "  z: List of z coordinates\n"
       "Returns:\n"
       "  Intrinsics");

    m.def("train", [&unwrap_result](const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z) {
        // Convert std::vector to AliceArray
        accurate_ri::PointCloud::Double cloud;
        cloud.x = accurate_ri::AliceArray<double>(x.data(), x.size());
        cloud.y = accurate_ri::AliceArray<double>(y.data(), y.size());
        cloud.z = accurate_ri::AliceArray<double>(z.data(), z.size());
        return unwrap_result(accurate_ri::train(cloud));
    }, py::arg("x"), py::arg("y"), py::arg("z"),
       "Estimate intrinsics from double vectors");

    m.def("debug_train", [&unwrap_result](const std::vector<float>& x, const std::vector<float>& y, const std::vector<float>& z) {
        // Convert std::vector to AliceArray
        accurate_ri::PointCloud::Float cloud;
        cloud.x = accurate_ri::AliceArray<float>(x.data(), x.size());
        cloud.y = accurate_ri::AliceArray<float>(y.data(), y.size());
        cloud.z = accurate_ri::AliceArray<float>(z.data(), z.size());
        return unwrap_result(accurate_ri::debugTrain(cloud));
    }, "Estimate intrinsics from float vectors with debug info");

    m.def("debug_train", [&unwrap_result](const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z) {
        // Convert std::vector to AliceArray
        accurate_ri::PointCloud::Double cloud;
        cloud.x = accurate_ri::AliceArray<double>(x.data(), x.size());
        cloud.y = accurate_ri::AliceArray<double>(y.data(), y.size());
        cloud.z = accurate_ri::AliceArray<double>(z.data(), z.size());
        return unwrap_result(accurate_ri::debugTrain(cloud));
    }, "Estimate intrinsics from double vectors with debug info");

    m.def("project_to_range_image", [&unwrap_result](const accurate_ri::Intrinsics& intr, const std::vector<float>& x, const std::vector<float>& y, const std::vector<float>& z) {
        // Convert std::vector to AliceArray
        accurate_ri::PointCloud::Float cloud;
        cloud.x = accurate_ri::AliceArray<float>(x.data(), x.size());
        cloud.y = accurate_ri::AliceArray<float>(y.data(), y.size());
        cloud.z = accurate_ri::AliceArray<float>(z.data(), z.size());
        return unwrap_result(accurate_ri::projectToRangeImage(intr, cloud));
    }, "Project float cloud to range image");

    m.def("project_to_range_image", [&unwrap_result](const accurate_ri::Intrinsics& intr, const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z) {
        // Convert std::vector to AliceArray
        accurate_ri::PointCloud::Double cloud;
        cloud.x = accurate_ri::AliceArray<double>(x.data(), x.size());
        cloud.y = accurate_ri::AliceArray<double>(y.data(), y.size());
        cloud.z = accurate_ri::AliceArray<double>(z.data(), z.size());
        return unwrap_result(accurate_ri::projectToRangeImage(intr, cloud));
    }, "Project double cloud to range image");

    m.def("unproject_to_point_cloud", [](const accurate_ri::Intrinsics& intr, const accurate_ri::RangeImage& ri) {
        auto cloud = accurate_ri::unProjectToPointCloud(intr, ri);
        // Convert AliceArray to std::vector for Python convenience
        std::vector<double> x_vec(cloud.x.begin(), cloud.x.end());
        std::vector<double> y_vec(cloud.y.begin(), cloud.y.end());
        std::vector<double> z_vec(cloud.z.begin(), cloud.z.end());
        return py::make_tuple(x_vec, y_vec, z_vec);
    }, "Unproject range image to 3D point cloud");

    // JSON functions
    m.def("intrinsics_to_json_str", [](const accurate_ri::Intrinsics& intrinsics, int32_t indent = -1) {
        auto result = accurate_ri::intrinsicsToJsonStr(intrinsics, indent);
        return std::string(result.c_str());
    }, py::arg("intrinsics"), py::arg("indent") = -1, "Convert intrinsics to JSON string");
    
    m.def("intrinsics_from_json_str", [&unwrap_result](const std::string& json) {
        return unwrap_result(accurate_ri::intrinsicsFromJsonStr(accurate_ri::AliceString(json.c_str())));
    }, py::arg("json"), "Create intrinsics from JSON string");

    m.def("intrinsics_to_json_file", [](const accurate_ri::Intrinsics& intrinsics, const std::string& output_path, int32_t indent = -1) {
        const auto status = accurate_ri::intrinsicsToJsonFile(intrinsics, output_path.c_str(), indent);
        if (!status) {
            throw std::runtime_error(std::string(status.message.c_str()));
        }
    }, py::arg("intrinsics"), py::arg("output_path"), py::arg("indent") = -1, "Write intrinsics to JSON file");

    m.def("intrinsics_from_json_file", [&unwrap_result](const std::string& path) {
        return unwrap_result(accurate_ri::intrinsicsFromJsonFile(path.c_str()));
    }, py::arg("path"), "Load intrinsics from JSON file");

    m.def("error_message", [](accurate_ri::ErrorCode code) {
        auto result = accurate_ri::errorMessage(code);
        return std::string(result.c_str());
    }, "Get error message for error code");
}
