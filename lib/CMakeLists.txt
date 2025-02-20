# TODO define release mode where we disable eigen debug and optimize the code for the native platform and so on

option(LOG_LEVEL "Set log level (DEBUG, INFO, WARN, ERROR, NONE)" "INFO")

# Convert human-readable log level to numeric values
if (LOG_LEVEL STREQUAL "DEBUG")
    add_definitions(-DLOG_LEVEL=1)
elseif (LOG_LEVEL STREQUAL "INFO")
    add_definitions(-DLOG_LEVEL=2)
elseif (LOG_LEVEL STREQUAL "WARN")
    add_definitions(-DLOG_LEVEL=3)
elseif (LOG_LEVEL STREQUAL "ERROR")
    add_definitions(-DLOG_LEVEL=4)
elseif (LOG_LEVEL STREQUAL "NONE") # Completely remove all logs
    add_definitions(-DLOG_LEVEL=5)
else ()
    message(FATAL_ERROR "Invalid LOG_LEVEL: Choose DEBUG, INFO, WARN, ERROR, or NONE")
endif ()

if (ENABLE_PROFILING)
    add_definitions(-DENABLE_PROFILING)
endif ()

if(ENABLE_TRACE_FILE)
    add_definitions(-DENABLE_TRACE_FILE)
    if (NOT LOG_LEVEL STREQUAL "DEBUG")
        message(FATAL_ERROR "LOG_LEVEL must be set to DEBUG when ENABLE_TRACE_FILE is enabled")
    endif()
endif()

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

#print out the EIGEN3_INCLUDE_DIR
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")

add_library(accurate_ri SHARED src/accurate_ri.cpp src/utils/Logger.h
        src/utils/Timer.h
        src/point/PointUtils.cpp
        src/point/PointUtils.h
        src/intrinsics/vertical/VerticalIntrinsicsEstimator.cpp
        src/intrinsics/vertical/VerticalIntrinsicsEstimator.h
        src/intrinsics/IntrinsicsEstimator.cpp
        src/intrinsics/IntrinsicsEstimator.h
        src/intrinsics/HorizontalIntrinsicsEstimator.cpp
        src/intrinsics/HorizontalIntrinsicsEstimator.h
        src/hough/HoughTransform.cpp
        src/hough/HoughTransform.h
        src/hash/HashUtils.cpp
        src/hash/HashUtils.h
        src/point/PointArray.h
        src/point/PointArray.cpp
        src/intrinsics/vertical/VerticalStructs.h
        src/utils/Utils.h
)

target_include_directories(accurate_ri
        PUBLIC include
        PRIVATE src
)

install(TARGETS accurate_ri DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)