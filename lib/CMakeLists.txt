option(LOG_LEVEL "Set log level (DEBUG, INFO, WARN, ERROR, NONE)" "INFO")

# Convert human-readable log level to numeric values
if(LOG_LEVEL STREQUAL "DEBUG")
    add_definitions(-DLOG_LEVEL=1)
elseif(LOG_LEVEL STREQUAL "INFO")
    add_definitions(-DLOG_LEVEL=2)
elseif(LOG_LEVEL STREQUAL "WARN")
    add_definitions(-DLOG_LEVEL=3)
elseif(LOG_LEVEL STREQUAL "ERROR")
    add_definitions(-DLOG_LEVEL=4)
elseif(LOG_LEVEL STREQUAL "NONE") # Completely remove all logs
    add_definitions(-DLOG_LEVEL=5)
else()
    message(FATAL_ERROR "Invalid LOG_LEVEL: Choose DEBUG, INFO, WARN, ERROR, or NONE")
endif()

if(ENABLE_PROFILING)
    add_definitions(-DENABLE_PROFILING)
endif()


add_library(accurate_ri SHARED src/accurate_ri.cpp src/utils/Logger.h
        src/utils/Timer.h
        src/point/PointUtils.cpp
        src/point/PointUtils.h
        src/intrinsics/VerticalIntrinsicsEstimator.cpp
        src/intrinsics/VerticalIntrinsicsEstimator.h
        src/intrinsics/IntrinsicsEstimator.cpp
        src/intrinsics/IntrinsicsEstimator.h
        src/intrinsics/HorizontalIntrinsicsEstimator.cpp
        src/intrinsics/HorizontalIntrinsicsEstimator.h
        src/hough/HoughTransform.cpp
        src/hough/HoughTransform.h
        src/hash/HashUtils.cpp
        src/hash/HashUtils.h)
target_include_directories(accurate_ri PUBLIC include)

install(TARGETS accurate_ri DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)