#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <accurate_ri/public_structs.hpp>
#include <accurate_ri/accurate_ri.hpp>

namespace py = pybind11;
using namespace accurate_ri;

PYBIND11_MODULE(_accurate_ri, m) {
    m.doc() = "Python bindings for the AccurateRI C++ library";

    // Enums
    py::enum_<EndReason>(m, "EndReason")
        .value("ALL_ASSIGNED", EndReason::ALL_ASSIGNED)
        .value("MAX_ITERATIONS", EndReason::MAX_ITERATIONS)
        .value("NO_MORE_PEAKS", EndReason::NO_MORE_PEAKS)
        .export_values();

    // Structs
    py::class_<RealMargin>(m, "RealMargin")
        .def(py::init<>())
        .def_readwrite("lower", &RealMargin::lower)
        .def_readwrite("upper", &RealMargin::upper)
        .def("diff", &RealMargin::diff)
        .def("clamp_both", &RealMargin::clampBoth);

    py::class_<ScanlineAngleBounds>(m, "ScanlineAngleBounds")
        .def(py::init<>())
        .def_readwrite("bottom", &ScanlineAngleBounds::bottom)
        .def_readwrite("top", &ScanlineAngleBounds::top);

    py::class_<OffsetAngleMargin>(m, "OffsetAngleMargin")
        .def(py::init<>())
        .def_readwrite("offset", &OffsetAngleMargin::offset)
        .def_readwrite("angle", &OffsetAngleMargin::angle);

    py::class_<OffsetAngle>(m, "OffsetAngle")
        .def(py::init<>())
        .def_readwrite("offset", &OffsetAngle::offset)
        .def_readwrite("angle", &OffsetAngle::angle);

    py::class_<ScanlineInfo>(m, "ScanlineInfo")
        .def(py::init<>())
        .def_readwrite("id", &ScanlineInfo::id)
        .def_readwrite("points_count", &ScanlineInfo::pointsCount)
        .def_readwrite("values", &ScanlineInfo::values)
        .def_readwrite("ci", &ScanlineInfo::ci)
        .def_readwrite("theoretical_angle_bounds", &ScanlineInfo::theoreticalAngleBounds)
        .def_readwrite("uncertainty", &ScanlineInfo::uncertainty)
        .def_readwrite("hough_votes", &ScanlineInfo::houghVotes)
        .def_readwrite("hough_hash", &ScanlineInfo::houghHash);

    py::class_<FullScanlines>(m, "FullScanlines")
        .def(py::init<>())
        .def_readwrite("scanlines", &FullScanlines::scanlines)
        .def_readwrite("points_scanlines_ids", &FullScanlines::pointsScanlinesIds);

    py::class_<ScanlineHorizontalInfo>(m, "ScanlineHorizontalInfo")
        .def(py::init<>())
        .def_readwrite("resolution", &ScanlineHorizontalInfo::resolution)
        .def_readwrite("offset", &ScanlineHorizontalInfo::offset)
        .def_readwrite("thetaOffset", &ScanlineHorizontalInfo::thetaOffset)
        .def_readwrite("heuristic", &ScanlineHorizontalInfo::heuristic);

    py::class_<HorizontalIntrinsicsResult>(m, "HorizontalIntrinsicsResult")
        .def(py::init<>())
        .def_readwrite("scanlines", &HorizontalIntrinsicsResult::scanlines);

    py::class_<VerticalIntrinsicsResult>(m, "VerticalIntrinsicsResult")
        .def(py::init<>())
        .def_readwrite("iterations", &VerticalIntrinsicsResult::iterations)
        .def_readwrite("scanlines_count", &VerticalIntrinsicsResult::scanlinesCount)
        .def_readwrite("unassigned_points", &VerticalIntrinsicsResult::unassignedPoints)
        .def_readwrite("points_count", &VerticalIntrinsicsResult::pointsCount)
        .def_readwrite("end_reason", &VerticalIntrinsicsResult::endReason)
        .def_readwrite("full_scanlines", &VerticalIntrinsicsResult::fullScanlines);

    py::class_<IntrinsicsResult>(m, "IntrinsicsResult")
        .def(py::init<>())
        .def_readwrite("vertical", &IntrinsicsResult::vertical)
        .def_readwrite("horizontal", &IntrinsicsResult::horizontal);


    py::class_<RangeImage>(m, "RangeImage")
        .def(py::init<uint32_t, uint32_t>(), py::arg("width"), py::arg("height"))
        .def(py::init<uint32_t, uint32_t, double>(), py::arg("width"), py::arg("height"), py::arg("initialValue"))
        .def_readonly("width", &RangeImage::width)
        .def_readonly("height", &RangeImage::height)
        .def("__getitem__", [](const RangeImage &ri, py::tuple idx) -> double {
            if (idx.size() != 2)
                throw py::index_error("Need 2 indices");
            size_t row = idx[0].cast<size_t>();
            size_t col = idx[1].cast<size_t>();
            if (row >= ri.height || col >= ri.width)
                throw py::index_error("Index out of bounds");
            return ri(row, col);
        }, py::is_operator())
        .def("__setitem__", [](RangeImage &ri, py::tuple idx, double value) {
            if (idx.size() != 2)
                throw py::index_error("Need 2 indices");
            size_t row = idx[0].cast<size_t>();
            size_t col = idx[1].cast<size_t>();
            if (row >= ri.height || col >= ri.width)
                throw py::index_error("Index out of bounds");
            ri(row, col) = value;
        }, py::is_operator())
        .def("__array__", [](py::object self) {
            auto& ri = self.cast<const RangeImage&>();
            // pybind11::array expects shape, strides, and pointer
            return py::array_t<double>(
                {ri.height, ri.width},                            // shape
                {sizeof(double) * ri.width, sizeof(double)},      // C-order strides
                &ri(0, 0),                                         // pointer to data
                self                                              // keep alive
            );
        });

    // Functions with vector inputs
    m.def("train", [](const std::vector<float>& x, const std::vector<float>& y, const std::vector<float>& z) {
        return train(PointCloud::Float{x, y, z});
    }, "Estimate intrinsics from float vectors");

    m.def("train", [](const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z) {
        return train(PointCloud::Double{x, y, z});
    }, "Estimate intrinsics from double vectors");

    m.def("project_to_range_image", [](const IntrinsicsResult& intr, const std::vector<float>& x, const std::vector<float>& y, const std::vector<float>& z) {
        return projectToRangeImage(intr, PointCloud::Float{x, y, z});
    }, "Project float cloud to range image");

    m.def("project_to_range_image", [](const IntrinsicsResult& intr, const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z) {
        return projectToRangeImage(intr, PointCloud::Double{x, y, z});
    }, "Project double cloud to range image");

    m.def("unproject_to_point_cloud", [](const IntrinsicsResult& intr, const RangeImage& ri) {
        auto cloud = unProjectToPointCloud(intr, ri);
        return py::make_tuple(cloud.x, cloud.y, cloud.z);
    }, "Unproject range image to 3D point cloud");

    m.def("write_to_json", &writeToJson, "Write intrinsics result to JSON");

    m.def("read_from_json", &readFromJson, "Read intrinsics result from JSON");
}
