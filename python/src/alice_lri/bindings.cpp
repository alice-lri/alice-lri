#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <string>
#include <vector>
#include <alice_lri/public_structs.hpp>
#include <alice_lri/alice_lri.hpp>
#include <alice_lri/Result.h>
#include <alice_lri/AliceString.h>
#include <sstream>
#include <iostream>

namespace py = pybind11;

PYBIND11_MODULE(_alice_lri, m) {
    m.doc() = "Python bindings for the ALICE-LRI C++ library";

    // Error handling
    py::enum_<alice_lri::ErrorCode>(m, "ErrorCode")
        .value("NONE", alice_lri::ErrorCode::NONE)
        .value("MISMATCHED_SIZES", alice_lri::ErrorCode::MISMATCHED_SIZES)
        .value("EMPTY_POINT_CLOUD", alice_lri::ErrorCode::EMPTY_POINT_CLOUD)
        .value("RANGES_XY_ZERO", alice_lri::ErrorCode::RANGES_XY_ZERO)
        .value("INTERNAL_ERROR", alice_lri::ErrorCode::INTERNAL_ERROR)
        .export_values();

    // Helper function to unwrap Result<T> and throw exceptions
    auto unwrap_result = [](const auto& result) -> auto {
        if (!result.ok()) {
            throw std::runtime_error(std::string(result.status().message.c_str()));
        }
        return result.value();
    };

    // Enums
    py::enum_<alice_lri::EndReason>(m, "EndReason")
        .value("ALL_ASSIGNED", alice_lri::EndReason::ALL_ASSIGNED)
        .value("MAX_ITERATIONS", alice_lri::EndReason::MAX_ITERATIONS)
        .value("NO_MORE_PEAKS", alice_lri::EndReason::NO_MORE_PEAKS)
        .export_values();

    // Core structs
    py::class_<alice_lri::Scanline>(m, "Scanline")
        .def(py::init<>())
        .def_readwrite("vertical_offset", &alice_lri::Scanline::verticalOffset)
        .def_readwrite("vertical_angle", &alice_lri::Scanline::verticalAngle)
        .def_readwrite("horizontal_offset", &alice_lri::Scanline::horizontalOffset)
        .def_readwrite("azimuthal_offset", &alice_lri::Scanline::azimuthalOffset)
        .def_readwrite("resolution", &alice_lri::Scanline::resolution)
        .def("__repr__", [](const alice_lri::Scanline& self) {
            std::ostringstream oss;
            oss << "Scanline(vertical_offset=" << self.verticalOffset
                << ", vertical_angle=" << self.verticalAngle
                << ", horizontal_offset=" << self.horizontalOffset
                << ", azimuthal_offset=" << self.azimuthalOffset
                << ", resolution=" << self.resolution << ")";
            return oss.str();
        });

    py::class_<alice_lri::Intrinsics>(m, "Intrinsics")
        .def(py::init<int32_t>())
        .def_property_readonly("scanlines", [](const alice_lri::Intrinsics& self) {
            return std::vector(self.scanlines.begin(), self.scanlines.end());
        })
        .def("__repr__", [](const alice_lri::Intrinsics& self) {
            std::ostringstream oss;
            oss << "Intrinsics(scanlines=[...])";
            return oss.str();
        });

    py::class_<alice_lri::Interval>(m, "Interval")
        .def(py::init<>())
        .def_readwrite("lower", &alice_lri::Interval::lower)
        .def_readwrite("upper", &alice_lri::Interval::upper)
        .def("diff", &alice_lri::Interval::diff)
        .def("any_contained", &alice_lri::Interval::anyContained)
        .def("clamp_both", &alice_lri::Interval::clampBoth)
        .def("__repr__", [](const alice_lri::Interval& self) {
            std::ostringstream oss;
            oss << "Interval(lower=" << self.lower << ", upper=" << self.upper << ")";
            return oss.str();
        });

    py::class_<alice_lri::ValueConfInterval>(m, "ValueConfInterval")
        .def(py::init<>())
        .def_readwrite("value", &alice_lri::ValueConfInterval::value)
        .def_readwrite("ci", &alice_lri::ValueConfInterval::ci)
        .def("__repr__", [](const alice_lri::ValueConfInterval& self) {
            std::ostringstream oss;
            oss << "ValueConfInterval(value=" << self.value << ", ci=Interval(lower=" << self.ci.lower << ", upper=" << self.ci.upper << "))";
            return oss.str();
        });

    py::class_<alice_lri::ScanlineAngleBounds>(m, "ScanlineAngleBounds")
        .def(py::init<>())
        .def_readwrite("lower_line", &alice_lri::ScanlineAngleBounds::lowerLine)
        .def_readwrite("upper_line", &alice_lri::ScanlineAngleBounds::upperLine)
        .def("__repr__", [](const alice_lri::ScanlineAngleBounds& self) {
            std::ostringstream oss;
            oss << "ScanlineAngleBounds(" <<
                "lower_line=[" << self.lowerLine.lower << ", " << self.lowerLine.upper << "], " <<
                "upper_line=[" << self.upperLine.lower << ", " << self.upperLine.upper << "])";
            return oss.str();
        });

    py::class_<alice_lri::DebugScanline>(m, "DebugScanline")
        .def(py::init<>())
        .def_readwrite("vertical_offset", &alice_lri::DebugScanline::verticalOffset)
        .def_readwrite("vertical_angle", &alice_lri::DebugScanline::verticalAngle)
        .def_readwrite("horizontal_offset", &alice_lri::DebugScanline::horizontalOffset)
        .def_readwrite("azimuthal_offset", &alice_lri::DebugScanline::azimuthalOffset)
        .def_readwrite("resolution", &alice_lri::DebugScanline::resolution)
        .def_readwrite("uncertainty", &alice_lri::DebugScanline::uncertainty)
        .def_readwrite("hough_votes", &alice_lri::DebugScanline::houghVotes)
        .def_readwrite("hough_hash", &alice_lri::DebugScanline::houghHash)
        .def_readwrite("points_count", &alice_lri::DebugScanline::pointsCount)
        .def_readwrite("theoretical_angle_bounds", &alice_lri::DebugScanline::theoreticalAngleBounds)
        .def_readwrite("vertical_heuristic", &alice_lri::DebugScanline::verticalHeuristic)
        .def_readwrite("horizontal_heuristic", &alice_lri::DebugScanline::horizontalHeuristic)
        .def("__repr__", [](const alice_lri::DebugScanline& self) {
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

    py::class_<alice_lri::DebugIntrinsics>(m, "DebugIntrinsics")
        .def(py::init<int32_t>())
        .def(py::init<int32_t, int32_t, int32_t, int32_t, alice_lri::EndReason>())
        .def_property_readonly("scanlines", [](const alice_lri::DebugIntrinsics& self) {
            return std::vector(self.scanlines.begin(), self.scanlines.end());
        })
        .def_readwrite("vertical_iterations", &alice_lri::DebugIntrinsics::verticalIterations)
        .def_readwrite("unassigned_points", &alice_lri::DebugIntrinsics::unassignedPoints)
        .def_readwrite("points_count", &alice_lri::DebugIntrinsics::pointsCount)
        .def_readwrite("end_reason", &alice_lri::DebugIntrinsics::endReason)
        .def("__repr__", [](const alice_lri::DebugIntrinsics& self) {
            std::ostringstream oss;
            oss << "DebugIntrinsics(scanlines=[" << self.scanlines.size() << "], vertical_iterations=" << self.verticalIterations
                << ", unassigned_points=" << self.unassignedPoints
                << ", points_count=" << self.pointsCount
                << ", end_reason=" << static_cast<int>(self.endReason) << ")";
            return oss.str();
        });

    // RangeImage class
    py::class_<alice_lri::RangeImage>(m, "RangeImage")
        .def(py::init<>())
        .def(py::init<uint32_t, uint32_t>(), py::arg("width"), py::arg("height"))
        .def(py::init<uint32_t, uint32_t, double>(), py::arg("width"), py::arg("height"), py::arg("initial_value"))
        .def_property_readonly("width", &alice_lri::RangeImage::width)
        .def_property_readonly("height", &alice_lri::RangeImage::height)
        .def("__repr__", [](const alice_lri::RangeImage& self) {
            std::ostringstream oss;
            oss << "RangeImage(width=" << self.width() << ", height=" << self.height() << ")";
            return oss.str();
        })
        .def("__getitem__", [](const alice_lri::RangeImage &ri, py::tuple idx) -> double {
            if (idx.size() != 2)
                throw py::index_error("Need 2 indices");
            size_t row = idx[0].cast<size_t>();
            size_t col = idx[1].cast<size_t>();
            if (row >= ri.height() || col >= ri.width())
                throw py::index_error("Index out of bounds");
            return ri(row, col);
        }, py::is_operator())
        .def("__setitem__", [](alice_lri::RangeImage &ri, py::tuple idx, double value) {
            if (idx.size() != 2)
                throw py::index_error("Need 2 indices");
            size_t row = idx[0].cast<size_t>();
            size_t col = idx[1].cast<size_t>();
            if (row >= ri.height() || col >= ri.width())
                throw py::index_error("Index out of bounds");
            ri(row, col) = value;
        }, py::is_operator())
        .def("__array__", [](py::object self) {
            auto& ri = self.cast<const alice_lri::RangeImage&>();
            return py::array_t<double>(
                {ri.height(), ri.width()},                        // shape
                {sizeof(double) * ri.width(), sizeof(double)},    // C-order strides
                ri.data(),                                         // pointer to data
                self                                              // keep alive
            );
        })
        .def("__repr__", [](const alice_lri::RangeImage& self) {
            std::ostringstream oss;
            oss << "RangeImage(width=" << self.width() << ", height=" << self.height() << ")";
            return oss.str();
        });

    // Main API functions with vector inputs for convenience
    m.def("train", [&unwrap_result](const std::vector<float>& x, const std::vector<float>& y, const std::vector<float>& z) {
        // Convert std::vector to AliceArray
        alice_lri::PointCloud::Float cloud;
        cloud.x = alice_lri::AliceArray<float>(x.data(), x.size());
        cloud.y = alice_lri::AliceArray<float>(y.data(), y.size());
        cloud.z = alice_lri::AliceArray<float>(z.data(), z.size());
        return unwrap_result(alice_lri::train(cloud));
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
        alice_lri::PointCloud::Double cloud;
        cloud.x = alice_lri::AliceArray<double>(x.data(), x.size());
        cloud.y = alice_lri::AliceArray<double>(y.data(), y.size());
        cloud.z = alice_lri::AliceArray<double>(z.data(), z.size());
        return unwrap_result(alice_lri::train(cloud));
    }, py::arg("x"), py::arg("y"), py::arg("z"),
       "Estimate intrinsics from double vectors");

    m.def("debug_train", [&unwrap_result](const std::vector<float>& x, const std::vector<float>& y, const std::vector<float>& z) {
        // Convert std::vector to AliceArray
        alice_lri::PointCloud::Float cloud;
        cloud.x = alice_lri::AliceArray<float>(x.data(), x.size());
        cloud.y = alice_lri::AliceArray<float>(y.data(), y.size());
        cloud.z = alice_lri::AliceArray<float>(z.data(), z.size());
        return unwrap_result(alice_lri::debugTrain(cloud));
    }, "Estimate intrinsics from float vectors with debug info");

    m.def("debug_train", [&unwrap_result](const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z) {
        // Convert std::vector to AliceArray
        alice_lri::PointCloud::Double cloud;
        cloud.x = alice_lri::AliceArray<double>(x.data(), x.size());
        cloud.y = alice_lri::AliceArray<double>(y.data(), y.size());
        cloud.z = alice_lri::AliceArray<double>(z.data(), z.size());
        return unwrap_result(alice_lri::debugTrain(cloud));
    }, "Estimate intrinsics from double vectors with debug info");

    m.def("project_to_range_image", [&unwrap_result](const alice_lri::Intrinsics& intr, const std::vector<float>& x, const std::vector<float>& y, const std::vector<float>& z) {
        // Convert std::vector to AliceArray
        alice_lri::PointCloud::Float cloud;
        cloud.x = alice_lri::AliceArray<float>(x.data(), x.size());
        cloud.y = alice_lri::AliceArray<float>(y.data(), y.size());
        cloud.z = alice_lri::AliceArray<float>(z.data(), z.size());
        return unwrap_result(alice_lri::projectToRangeImage(intr, cloud));
    }, "Project float cloud to range image");

    m.def("project_to_range_image", [&unwrap_result](const alice_lri::Intrinsics& intr, const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z) {
        // Convert std::vector to AliceArray
        alice_lri::PointCloud::Double cloud;
        cloud.x = alice_lri::AliceArray<double>(x.data(), x.size());
        cloud.y = alice_lri::AliceArray<double>(y.data(), y.size());
        cloud.z = alice_lri::AliceArray<double>(z.data(), z.size());
        return unwrap_result(alice_lri::projectToRangeImage(intr, cloud));
    }, "Project double cloud to range image");

    m.def("unproject_to_point_cloud", [](const alice_lri::Intrinsics& intr, const alice_lri::RangeImage& ri) {
        auto cloud = alice_lri::unProjectToPointCloud(intr, ri);
        // Convert AliceArray to std::vector for Python convenience
        std::vector<double> x_vec(cloud.x.begin(), cloud.x.end());
        std::vector<double> y_vec(cloud.y.begin(), cloud.y.end());
        std::vector<double> z_vec(cloud.z.begin(), cloud.z.end());
        return py::make_tuple(x_vec, y_vec, z_vec);
    }, "Unproject range image to 3D point cloud");

    // JSON functions
    m.def("intrinsics_to_json_str", [](const alice_lri::Intrinsics& intrinsics, int32_t indent = -1) {
        auto result = alice_lri::intrinsicsToJsonStr(intrinsics, indent);
        return std::string(result.c_str());
    }, py::arg("intrinsics"), py::arg("indent") = -1, "Convert intrinsics to JSON string");
    
    m.def("intrinsics_from_json_str", [&unwrap_result](const std::string& json) {
        return unwrap_result(alice_lri::intrinsicsFromJsonStr(alice_lri::AliceString(json.c_str())));
    }, py::arg("json"), "Create intrinsics from JSON string");

    m.def("intrinsics_to_json_file", [](const alice_lri::Intrinsics& intrinsics, const std::string& output_path, int32_t indent = -1) {
        const auto status = alice_lri::intrinsicsToJsonFile(intrinsics, output_path.c_str(), indent);
        if (!status) {
            throw std::runtime_error(std::string(status.message.c_str()));
        }
    }, py::arg("intrinsics"), py::arg("output_path"), py::arg("indent") = -1, "Write intrinsics to JSON file");

    m.def("intrinsics_from_json_file", [&unwrap_result](const std::string& path) {
        return unwrap_result(alice_lri::intrinsicsFromJsonFile(path.c_str()));
    }, py::arg("path"), "Load intrinsics from JSON file");

    m.def("error_message", [](alice_lri::ErrorCode code) {
        auto result = alice_lri::errorMessage(code);
        return std::string(result.c_str());
    }, "Get error message for error code");
}
