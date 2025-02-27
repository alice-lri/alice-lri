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
    if (NOT LOG_LEVEL STREQUAL "INFO")
        message(FATAL_ERROR "LOG_LEVEL must be set to INFO when ENABLE_TRACE_FILE is enabled")
    endif()
endif()

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

#print out the EIGEN3_INCLUDE_DIR
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")

set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(external/json)

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
        src/intrinsics/vertical/helper/VerticalLogging.h
        src/utils/TestUtils.h
        src/intrinsics/vertical/helper/VerticalLogging.cpp
        src/intrinsics/vertical/helper/JsonConverters.cpp
        src/intrinsics/vertical/helper/JsonConverters.h
        src/math/Stats.cpp
        src/math/Stats.h
        src/intrinsics/vertical/conflict/ScanlineConflictSolver.cpp
        src/intrinsics/vertical/conflict/ScanlineConflictSolver.h
        src/intrinsics/vertical/pool/VerticalScanlinePool.cpp
        src/intrinsics/vertical/pool/VerticalScanlinePool.h
)

target_include_directories(accurate_ri
        PUBLIC include
        PRIVATE src
)

target_link_libraries(accurate_ri PRIVATE nlohmann_json::nlohmann_json)

if(ENABLE_STRICT_WARNINGS) #TODO make this trigger in release mode
    message(STATUS "Strict warnings enabled")
    target_compile_options(accurate_ri PRIVATE
            -Wall -Wextra -Wpedantic  # General best practices
            -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align
            -Woverloaded-virtual -Wconversion -Wsign-conversion
            -Wpessimizing-move -Wredundant-move -Werror=unused-value
            -Werror=unused-variable -Werror=unused-parameter
    )
endif()

install(TARGETS accurate_ri DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)