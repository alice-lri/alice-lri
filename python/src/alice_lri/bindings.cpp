#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <string>
#include <vector>
#include <alice_lri/Core.hpp>
#include <sstream>
#include <iostream>

namespace py = pybind11;

PYBIND11_MODULE(_alice_lri, m) {
    m.doc() = "Python bindings for the ALICE-LRI C++ library";

    // Error handling
    py::enum_<alice_lri::ErrorCode>(m, "ErrorCode", R"doc(
        Error codes for Alice LRI operations.
    )doc")
        .value("NONE", alice_lri::ErrorCode::NONE, "No error.")
        .value("MISMATCHED_SIZES", alice_lri::ErrorCode::MISMATCHED_SIZES, "Input arrays have mismatched sizes.")
        .value("EMPTY_POINT_CLOUD", alice_lri::ErrorCode::EMPTY_POINT_CLOUD, "Point cloud is empty.")
        .value("RANGES_XY_ZERO", alice_lri::ErrorCode::RANGES_XY_ZERO, "At least one point has a range of zero in the XY plane.")
        .value("INTERNAL_ERROR", alice_lri::ErrorCode::INTERNAL_ERROR, "Internal error occurred.")
        .export_values();

    // Helper function to unwrap Result<T> and throw exceptions
    auto unwrap_result = [](const auto& result) -> auto {
        if (!result.ok()) {
            throw std::runtime_error(std::string(result.status().message.c_str()));
        }
        return result.value();
    };

    // Enums
    py::enum_<alice_lri::EndReason>(m, "EndReason", R"doc(
        Reason for ending the iterative vertical fitting process.
    )doc")
        .value("ALL_ASSIGNED", alice_lri::EndReason::ALL_ASSIGNED, "All points assigned. This is the normal termination condition.")
        .value("MAX_ITERATIONS", alice_lri::EndReason::MAX_ITERATIONS, "Maximum number of iterations reached.")
        .value("NO_MORE_PEAKS", alice_lri::EndReason::NO_MORE_PEAKS, "No more peaks found in the Hough accumulator.")
        .export_values();

    // Core structs
    py::class_<alice_lri::Scanline>(m, "Scanline", R"doc(
        Represents a single scanline with intrinsic parameters.

        Attributes:
            vertical_offset (float): Vertical spatial offset of the scanline.
            vertical_angle (float): Vertical angle of the scanline.
            horizontal_offset (float): Horizontal spatial offset of the scanline.
            azimuthal_offset (float): Azimuthal offset of the scanline.
            resolution (int): Horizontal resolution of the scanline.
    )doc")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite("vertical_offset", &alice_lri::Scanline::verticalOffset, "Vertical spatial offset.")
        .def_readwrite("vertical_angle", &alice_lri::Scanline::verticalAngle, "Vertical angle.")
        .def_readwrite("horizontal_offset", &alice_lri::Scanline::horizontalOffset, "Horizontal spatial offset.")
        .def_readwrite("azimuthal_offset", &alice_lri::Scanline::azimuthalOffset, "Azimuthal offset.")
        .def_readwrite("resolution", &alice_lri::Scanline::resolution, "Horizontal resolution.")
        .def("__repr__", [](const alice_lri::Scanline& self) {
            std::ostringstream oss;
            oss << "Scanline(vertical_offset=" << self.verticalOffset
                << ", vertical_angle=" << self.verticalAngle
                << ", horizontal_offset=" << self.horizontalOffset
                << ", azimuthal_offset=" << self.azimuthalOffset
                << ", resolution=" << self.resolution << ")";
            return oss.str();
        });

    py::class_<alice_lri::Intrinsics>(m, "Intrinsics", R"doc(
        Contains intrinsic parameters for a sensor, including all scanlines.

        Args:
            scanline_count (int): Number of scanlines.

        Attributes:
            scanlines (list of Scanline): Array of scanlines describing the sensor geometry.
    )doc")
        .def(py::init<int32_t>(), py::arg("scanline_count"), "Construct with a given number of scanlines.")
        .def_property_readonly("scanlines", [](const alice_lri::Intrinsics& self) {
            return std::vector(self.scanlines.begin(), self.scanlines.end());
        }, "List of scanlines.")
        .def("__repr__", [](const alice_lri::Intrinsics& self) {
            std::ostringstream oss;
            oss << "Intrinsics(scanlines=[...])";
            return oss.str();
        });

    py::class_<alice_lri::Interval>(m, "Interval", R"doc(
        Represents a numeric interval [lower, upper].

        Attributes:
            lower (float): Lower bound of the interval.
            upper (float): Upper bound of the interval.
    )doc")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite("lower", &alice_lri::Interval::lower, "Lower bound.")
        .def_readwrite("upper", &alice_lri::Interval::upper, "Upper bound.")
        .def("diff", &alice_lri::Interval::diff, "Get the width of the interval (upper - lower).")
        .def("any_contained", &alice_lri::Interval::anyContained, "Check if any part of another interval is contained in this interval.")
        .def("clamp_both", &alice_lri::Interval::clampBoth, "Clamp both bounds to [minValue, maxValue].")
        .def("__repr__", [](const alice_lri::Interval& self) {
            std::ostringstream oss;
            oss << "Interval(lower=" << self.lower << ", upper=" << self.upper << ")";
            return oss.str();
        });

    py::class_<alice_lri::ValueConfInterval>(m, "ValueConfInterval", R"doc(
        Value with associated confidence interval.

        Attributes:
            value (float): The value.
            ci (Interval): Confidence interval for the value.
    )doc")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite("value", &alice_lri::ValueConfInterval::value, "The value.")
        .def_readwrite("ci", &alice_lri::ValueConfInterval::ci, "Confidence interval.")
        .def("__repr__", [](const alice_lri::ValueConfInterval& self) {
            std::ostringstream oss;
            oss << "ValueConfInterval(value=" << self.value << ", ci=Interval(lower=" << self.ci.lower << ", upper=" << self.ci.upper << "))";
            return oss.str();
        });

    py::class_<alice_lri::ScanlineAngleBounds>(m, "ScanlineAngleBounds", R"doc(
        Angle bounds for a scanline.

        Attributes:
            lower_line (Interval): Lower angle interval.
            upper_line (Interval): Upper angle interval.
    )doc")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite("lower_line", &alice_lri::ScanlineAngleBounds::lowerLine, "Lower angle interval.")
        .def_readwrite("upper_line", &alice_lri::ScanlineAngleBounds::upperLine, "Upper angle interval.")
        .def("__repr__", [](const alice_lri::ScanlineAngleBounds& self) {
            std::ostringstream oss;
            oss << "ScanlineAngleBounds(" <<
                "lower_line=[" << self.lowerLine.lower << ", " << self.lowerLine.upper << "], " <<
                "upper_line=[" << self.upperLine.lower << ", " << self.upperLine.upper << "])";
            return oss.str();
        });

    py::class_<alice_lri::ScanlineDetailed>(m, "ScanlineDetailed", R"doc(
        Detailed scanline information with uncertainty and voting statistics.

        Attributes:
            vertical_offset (ValueConfInterval): Vertical spatial offset with confidence interval.
            vertical_angle (ValueConfInterval): Vertical angle with confidence interval.
            horizontal_offset (float): Horizontal spatial offset.
            azimuthal_offset (float): Azimuthal offset.
            resolution (int): Horizontal resolution of the scanline.
            uncertainty (float): Estimated uncertainty.
            hough_votes (int): Number of Hough transform votes.
            hough_hash (int): Hash value for Hough voting.
            points_count (int): Number of points assigned to this scanline.
            theoretical_angle_bounds (ScanlineAngleBounds): Theoretical angle bounds for the scanline.
            vertical_heuristic (bool): Whether vertical heuristic was used.
            horizontal_heuristic (bool): Whether horizontal heuristic was used.
    )doc")
        .def(py::init<>(), "Default constructor.")
        .def_readwrite("vertical_offset", &alice_lri::ScanlineDetailed::verticalOffset, "Vertical offset with confidence interval.")
        .def_readwrite("vertical_angle", &alice_lri::ScanlineDetailed::verticalAngle, "Vertical angle with confidence interval.")
        .def_readwrite("horizontal_offset", &alice_lri::ScanlineDetailed::horizontalOffset, "Horizontal offset.")
        .def_readwrite("azimuthal_offset", &alice_lri::ScanlineDetailed::azimuthalOffset, "Azimuthal offset.")
        .def_readwrite("resolution", &alice_lri::ScanlineDetailed::resolution, "Number of points in the scanline.")
        .def_readwrite("uncertainty", &alice_lri::ScanlineDetailed::uncertainty, "Estimated uncertainty.")
        .def_readwrite("hough_votes", &alice_lri::ScanlineDetailed::houghVotes, "Number of Hough transform votes.")
        .def_readwrite("hough_hash", &alice_lri::ScanlineDetailed::houghHash, "Hash value for Hough voting.")
        .def_readwrite("points_count", &alice_lri::ScanlineDetailed::pointsCount, "Number of points assigned to this scanline.")
        .def_readwrite("theoretical_angle_bounds", &alice_lri::ScanlineDetailed::theoreticalAngleBounds, "Theoretical angle bounds.")
        .def_readwrite("vertical_heuristic", &alice_lri::ScanlineDetailed::verticalHeuristic, "Whether vertical heuristic was used.")
        .def_readwrite("horizontal_heuristic", &alice_lri::ScanlineDetailed::horizontalHeuristic, "Whether horizontal heuristic was used.")
        .def("__repr__", [](const alice_lri::ScanlineDetailed& self) {
            std::ostringstream oss;
            oss << "ScanlineDetailed(vertical_offset=" << self.verticalOffset.value
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

    py::class_<alice_lri::IntrinsicsDetailed>(m, "IntrinsicsDetailed", R"doc(
        Detailed intrinsic parameters, including scanline details and statistics.

        Args:
            scanline_count (int): Number of scanlines.
            vertical_iterations (int): Number of vertical iterations performed.
            unassigned_points (int): Number of unassigned points.
            points_count (int): Total number of points.
            end_reason (EndReason): Reason for ending the process.

        Attributes:
            scanlines (list of ScanlineDetailed): List of detailed scanlines.
            vertical_iterations (int): Number of vertical iterations performed.
            unassigned_points (int): Number of unassigned points.
            points_count (int): Total number of points.
            end_reason (EndReason): Reason for ending the process.
    )doc")
        .def(py::init<int32_t>(), py::arg("scanline_count"), "Construct with a given number of scanlines.")
        .def(py::init<int32_t, int32_t, int32_t, int32_t, alice_lri::EndReason>(),
            py::arg("scanline_count"), py::arg("vertical_iterations"), py::arg("unassigned_points"), py::arg("points_count"), py::arg("end_reason"),
            "Full constructor with all statistics.")
        .def_property_readonly("scanlines", [](const alice_lri::IntrinsicsDetailed& self) {
            return std::vector(self.scanlines.begin(), self.scanlines.end());
        }, "List of detailed scanlines.")
        .def_readwrite("vertical_iterations", &alice_lri::IntrinsicsDetailed::verticalIterations, "Number of vertical iterations performed.")
        .def_readwrite("unassigned_points", &alice_lri::IntrinsicsDetailed::unassignedPoints, "Number of unassigned points.")
        .def_readwrite("points_count", &alice_lri::IntrinsicsDetailed::pointsCount, "Total number of points.")
        .def_readwrite("end_reason", &alice_lri::IntrinsicsDetailed::endReason, "Reason for ending the process.")
        .def("__repr__", [](const alice_lri::IntrinsicsDetailed& self) {
            std::ostringstream oss;
            oss << "IntrinsicsDetailed(scanlines=[" << self.scanlines.size() << "], vertical_iterations=" << self.verticalIterations
                << ", unassigned_points=" << self.unassignedPoints
                << ", points_count=" << self.pointsCount
                << ", end_reason=" << static_cast<int>(self.endReason) << ")";
            return oss.str();
        });

    // RangeImage class
    py::class_<alice_lri::RangeImage>(m, "RangeImage", R"doc(
        Represents a 2D range image with pixel data.

        Args:
            width (int): Image width.
            height (int): Image height.
            initial_value (float, optional): Initial value for all pixels (if provided).

        Attributes:
            width (int): Image width.
            height (int): Image height.

        Note:
            The (width, height) constructor only reserves space for pixels but does not initialize them.
            The (width, height, initial_value) constructor initializes all pixels to the given value.
    )doc")
        .def(py::init<>(), "Default constructor (empty image).")
        .def(py::init<uint32_t, uint32_t>(), py::arg("width"), py::arg("height"), "Construct with width and height. Reserves space for pixels but does not initialize them.")
        .def(py::init<uint32_t, uint32_t, double>(), py::arg("width"), py::arg("height"), py::arg("initial_value"), "Construct with width, height, and initial pixel value.")
        .def_property_readonly("width", &alice_lri::RangeImage::width, "Get image width.")
        .def_property_readonly("height", &alice_lri::RangeImage::height, "Get image height.")
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
        .def("__array__", [](py::object self, py::kwargs kwargs) {
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

    m.def("estimate_intrinsics", [&unwrap_result](const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z) {
        // Convert std::vector to AliceArray
        alice_lri::PointCloud::Double cloud;
        cloud.x = alice_lri::AliceArray<double>(x.data(), x.size());
        cloud.y = alice_lri::AliceArray<double>(y.data(), y.size());
        cloud.z = alice_lri::AliceArray<double>(z.data(), z.size());
        return unwrap_result(alice_lri::estimateIntrinsics(cloud));
    }, py::arg("x"), py::arg("y"), py::arg("z"),
       R"doc(
        Estimate sensor intrinsics from point cloud coordinates given as float vectors.

        Args:
            x (list of float): X coordinates.
            y (list of float): Y coordinates.
            z (list of float): Z coordinates.
        Returns:
            Intrinsics: Estimated sensor intrinsics.
    )doc");

    m.def("estimate_intrinsics_detailed", [&unwrap_result](const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z) {
        // Convert std::vector to AliceArray
        alice_lri::PointCloud::Double cloud;
        cloud.x = alice_lri::AliceArray<double>(x.data(), x.size());
        cloud.y = alice_lri::AliceArray<double>(y.data(), y.size());
        cloud.z = alice_lri::AliceArray<double>(z.data(), z.size());
        return unwrap_result(alice_lri::estimateIntrinsicsDetailed(cloud));
    }, py::arg("x"), py::arg("y"), py::arg("z"), R"doc(
        Estimate detailed sensor intrinsics (including algorithm execution info) from point cloud coordinates given as float vectors.

        Args:
            x (list of float): X coordinates.
            y (list of float): Y coordinates.
            z (list of float): Z coordinates.
        Returns:
            IntrinsicsDetailed: Detailed estimated intrinsics and statistics.
    )doc");

    m.def("project_to_range_image", [&unwrap_result](
        const alice_lri::Intrinsics& intrinsics, const std::vector<double>& x, const std::vector<double>& y,
        const std::vector<double>& z
    ) {
        // Convert std::vector to AliceArray
        alice_lri::PointCloud::Double cloud;
        cloud.x = alice_lri::AliceArray<double>(x.data(), x.size());
        cloud.y = alice_lri::AliceArray<double>(y.data(), y.size());
        cloud.z = alice_lri::AliceArray<double>(z.data(), z.size());
        return unwrap_result(alice_lri::projectToRangeImage(intrinsics, cloud));
    }, py::arg("intrinsics"), py::arg("x"), py::arg("y"), py::arg("z"), R"doc(
        Project a point cloud to a range image using given intrinsics.

        Args:
            intrinsics (Intrinsics): Sensor intrinsics (see estimate_intrinsics).
            x (list of float): X coordinates.
            y (list of float): Y coordinates.
            z (list of float): Z coordinates.
        Returns:
            RangeImage: Projected range image.
    )doc");

    m.def("unproject_to_point_cloud", [](const alice_lri::Intrinsics& intrinsics, const alice_lri::RangeImage& ri) {
        auto cloud = alice_lri::unProjectToPointCloud(intrinsics, ri);
        // Convert AliceArray to std::vector for Python convenience
        std::vector<double> x_vec(cloud.x.begin(), cloud.x.end());
        std::vector<double> y_vec(cloud.y.begin(), cloud.y.end());
        std::vector<double> z_vec(cloud.z.begin(), cloud.z.end());
        return py::make_tuple(x_vec, y_vec, z_vec);
    }, py::arg("intrinsics"), py::arg("ri"), R"doc(
        Unproject a range image to a 3D point cloud using given intrinsics.

        Args:
            intrinsics (Intrinsics): Sensor intrinsics.
            ri (RangeImage): Input range image.
        Returns:
            tuple: (x, y, z) coordinate lists.
    )doc");

    // JSON functions
    m.def("intrinsics_to_json_str", [](const alice_lri::Intrinsics& intrinsics, int32_t indent = -1) {
        auto result = alice_lri::intrinsicsToJsonStr(intrinsics, indent);
        return std::string(result.c_str());
    }, py::arg("intrinsics"), py::arg("indent") = -1,
       R"doc(
        Convert intrinsics to a JSON string.

        Args:
            intrinsics (Intrinsics): Intrinsics to serialize.
            indent (int, optional): Indentation for pretty printing (-1 for compact).
        Returns:
            str: JSON string.
    )doc");
    
    m.def("intrinsics_from_json_str", [&unwrap_result](const std::string& json) {
        return unwrap_result(alice_lri::intrinsicsFromJsonStr(alice_lri::AliceString(json.c_str())));
    }, py::arg("json"),
       R"doc(
        Create intrinsics from a JSON string.

        Args:
            json (str): JSON string.
        Returns:
            Intrinsics: Parsed intrinsics.
    )doc");

    m.def("intrinsics_to_json_file", [](const alice_lri::Intrinsics& intrinsics, const std::string& output_path, int32_t indent = -1) {
        const auto status = alice_lri::intrinsicsToJsonFile(intrinsics, output_path.c_str(), indent);
        if (!status) {
            throw std::runtime_error(std::string(status.message.c_str()));
        }
    }, py::arg("intrinsics"), py::arg("output_path"), py::arg("indent") = -1,
       R"doc(
        Write intrinsics to a JSON file.

        Args:
            intrinsics (Intrinsics): Intrinsics to write.
            output_path (str): Output file path.
            indent (int, optional): Indentation for pretty printing (-1 for compact).
        Raises:
            RuntimeError: If writing fails.
    )doc");

    m.def("intrinsics_from_json_file", [&unwrap_result](const std::string& path) {
        return unwrap_result(alice_lri::intrinsicsFromJsonFile(path.c_str()));
    }, py::arg("path"),
       R"doc(
        Load intrinsics from a JSON file.

        Args:
            path (str): Path to JSON file.
        Returns:
            Intrinsics: Parsed intrinsics.
    )doc");

    m.def("error_message", [](alice_lri::ErrorCode code) {
        auto result = alice_lri::errorMessage(code);
        return std::string(result.c_str());
    }, py::arg("code"), R"doc(
        Get a human-readable error message for an error code.

        Args:
            code (ErrorCode): Error code.
        Returns:
            str: Error message.
    )doc");
}
