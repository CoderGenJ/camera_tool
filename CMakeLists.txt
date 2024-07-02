cmake_minimum_required(VERSION 3.10)

# 设置项目名称和版本
project(camera_tool VERSION 1.0)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找OpenCV库
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV version ${OpenCV_VERSION}")
else()
    message(FATAL_ERROR "OpenCV not found")
endif()

# 查找Eigen库
find_package(Eigen3 REQUIRED)
if (Eigen3_FOUND)
    message(STATUS "Found Eigen3")
else()
    message(FATAL_ERROR "Eigen3 not found")
endif()

find_package(Ceres REQUIRED)
find_package(GTest REQUIRED)
# 包含OpenCV头文件路径
include_directories(${OpenCV_INCLUDE_DIRS})
# 包含Eigen头文件路径
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CERES_INCLUDE_DIR})
include_directories(${GTEST_INCLUDE_DIRS})

# 添加子目录
add_subdirectory(tools)
add_subdirectory(camera_model)
add_subdirectory(calibration_model)
add_subdirectory(factor)
add_subdirectory(common)
add_subdirectory(marker_detector)
add_subdirectory(pnp_solver)
add_subdirectory(3rparty)
add_subdirectory(sfm)

# 打印消息以确认包含的目录
message(STATUS "Include directories: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "Eigen3 include directory: ${EIGEN3_INCLUDE_DIR}")
