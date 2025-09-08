# TODO define release mode where we disable eigen debug and optimize the code for the native platform and so on
set(CMAKE_CXX_STANDARD 20)
option(LOG_LEVEL "Set log level (DEBUG, INFO, WARN, ERROR, NONE)" "INFO")

if (LOG_LEVEL STREQUAL "DEBUG")
    add_definitions(-DLOG_LEVEL=1)
elseif (LOG_LEVEL STREQUAL "INFO")
    add_definitions(-DLOG_LEVEL=2)
elseif (LOG_LEVEL STREQUAL "WARN")
    add_definitions(-DLOG_LEVEL=3)
elseif (LOG_LEVEL STREQUAL "ERROR")
    add_definitions(-DLOG_LEVEL=4)
elseif (LOG_LEVEL STREQUAL "NONE")
    add_definitions(-DLOG_LEVEL=5)
else ()
    message(FATAL_ERROR "Invalid LOG_LEVEL: Choose DEBUG, INFO, WARN, ERROR, or NONE")
endif ()

option(FLAG_USE_HOUGH_CONTINUITY "Set BuildOption USE_HOUGH_CONTINUITY" ON)
option(FLAG_USE_SCANLINE_CONFLICT_SOLVER "Set BuildOption USE_SCANLINE_CONFLICT_SOLVER" ON)
option(FLAG_USE_VERTICAL_HEURISTICS "Set BuildOption USE_VERTICAL_HEURISTICS" ON)
option(FLAG_USE_HORIZONTAL_HEURISTICS "Set BuildOption USE_HORIZONTAL_HEURISTICS" ON)

configure_file("src/BuildOptions.h.in" "src/BuildOptions.h")

if (ENABLE_PROFILING)
    add_definitions(-DENABLE_PROFILING)
endif ()

if (ENABLE_TRACE_FILE)
    add_definitions(-DENABLE_TRACE_FILE)
endif ()

option(LIB_MODE "Build in lib mode, hiding unnecessary symbols for ABI compatibility" OFF)

if (LIB_MODE)
    set(CMAKE_CXX_VISIBILITY_PRESET hidden)
    set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)
else()
    set(ENABLE_PYTHON_DEBUG ON)
    add_definitions(-DENABLE_PYTHON_DEBUG)
endif ()

include(${CMAKE_BINARY_DIR}/lib/conan_toolchain.cmake)

find_package(Eigen3 REQUIRED)
include_directories(Eigen3_INCLUDE_DIRS)

message(STATUS "EIGEN3_INCLUDE_DIR: ${Eigen3_INCLUDE_DIRS}")

find_package(nlohmann_json REQUIRED)
set(JSON_BuildTests OFF CACHE INTERNAL "")

message(STATUS "osqp_INCLUDE_DIRS: ${osqp_INCLUDE_DIRS}")

if (ENABLE_PYTHON_DEBUG)
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

    message(STATUS "Python3 include dir: ${Python3_INCLUDE_DIRS}")
    message(STATUS "Python3 library: ${Python3_LIBRARIES}")
endif ()

add_library(accurate_ri SHARED src/accurate_ri.cpp src/utils/Logger.h
        src/utils/Timer.h
        src/point/PointUtils.cpp
        src/point/PointUtils.h
        src/intrinsics/vertical/VerticalIntrinsicsEstimator.cpp
        src/intrinsics/vertical/VerticalIntrinsicsEstimator.h
        src/intrinsics/IntrinsicsEstimator.cpp
        src/intrinsics/IntrinsicsEstimator.h
        src/hough/HoughTransform.cpp
        src/hough/HoughTransform.h
        src/hash/HashUtils.cpp
        src/hash/HashUtils.h
        src/point/PointArray.h
        src/point/PointArray.cpp
        src/intrinsics/vertical/VerticalStructs.h
        src/utils/Utils.h
        src/intrinsics/vertical/helper/VerticalLogging.h
        src/utils/TestUtils.h
        src/utils/json/JsonConverters.cpp
        src/utils/json/JsonConverters.h
        src/math/Stats.cpp
        src/math/Stats.h
        src/intrinsics/vertical/conflict/ScanlineConflictSolver.cpp
        src/intrinsics/vertical/conflict/ScanlineConflictSolver.h
        src/intrinsics/vertical/pool/VerticalScanlinePool.cpp
        src/intrinsics/vertical/pool/VerticalScanlinePool.h
        src/intrinsics/horizontal/helper/HorizontalScanlineArray.cpp
        src/intrinsics/horizontal/helper/HorizontalScanlineArray.h
        src/intrinsics/horizontal/helper/HorizontalMath.cpp
        src/intrinsics/horizontal/helper/HorizontalMath.h
        include/accurate_ri/public_structs.hpp
        src/intrinsics/horizontal/HorizontalIntrinsicsEstimator.cpp
        src/intrinsics/horizontal/HorizontalIntrinsicsEstimator.h
        src/plotty/matplotlibcpp.hpp
        src/plotty/matplotlibcpp.cpp
        src/rangeimage/RangeImageUtils.cpp
        src/rangeimage/RangeImageUtils.h
        src/Constants.h
        src/intrinsics/horizontal/helper/SegmentedMedianLinearRegressor.cpp
        src/intrinsics/horizontal/helper/SegmentedMedianLinearRegressor.h
        src/public_structs.cpp
        src/intrinsics/vertical/estimation/VerticalScanlineLimits.cpp
        src/intrinsics/vertical/estimation/VerticalScanlineLimits.h
        src/intrinsics/vertical/estimation/VerticalScanlineEstimator.cpp
        src/intrinsics/vertical/estimation/VerticalScanlineEstimator.h
        src/intrinsics/horizontal/helper/PeriodicFitter.cpp
        src/intrinsics/horizontal/helper/PeriodicFitter.h
        src/math/Trigonometry.h
        src/math/Trigonometry.cpp
        src/intrinsics/vertical/estimation/VerticalHeuristicsEstimator.cpp
        src/intrinsics/vertical/estimation/VerticalHeuristicsEstimator.h
)

set(_accurate_ri_private_include_dirs
        src
        "${CMAKE_CURRENT_BINARY_DIR}/src"
        ${nlohmann_json_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
)

set(_accurate_ri_private_libraries
        nlohmann_json::nlohmann_json
)

if (ENABLE_PYTHON_DEBUG)
    list(APPEND _accurate_ri_private_include_dirs ${Python3_INCLUDE_DIRS})
    list(APPEND _accurate_ri_private_libraries ${Python3_LIBRARIES})
endif ()

target_include_directories(accurate_ri
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE
        ${_accurate_ri_private_include_dirs}
)

target_link_libraries(accurate_ri PRIVATE ${_accurate_ri_private_libraries})


if (ENABLE_STRICT_WARNINGS) #TODO make this trigger in release mode
    message(STATUS "Strict warnings enabled")
    target_compile_options(accurate_ri PRIVATE
            -Wall -Wextra -Wpedantic  # General best practices
            -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align
            -Woverloaded-virtual -Wconversion -Wsign-conversion
            -Wpessimizing-move -Wredundant-move -Werror=unused-value
            -Werror=unused-variable -Werror=unused-parameter
    )
endif ()

# Installation
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

install(TARGETS accurate_ri
        EXPORT accurate_riTargets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT accurate_riTargets
        FILE accurate_riTargets.cmake
        NAMESPACE accurate_ri::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/accurate_ri
)

write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/accurate_riConfigVersion.cmake"
        VERSION 1.0.0
        COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/accurate_riConfig.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/accurate_riConfig.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/accurate_ri
)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/accurate_riConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/accurate_riConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/accurate_ri
)
