# 创建一个库或者可执行文件
include_directories(${CMAKE_SOURCE_DIR}/common)
include_directories(${CMAKE_SOURCE_DIR}/sfm)
include_directories(${CMAKE_SOURCE_DIR}/camera_model)
include_directories(${CMAKE_SOURCE_DIR}/calibration_model)
include_directories(${CMAKE_SOURCE_DIR}/factor)
include_directories(${CMAKE_SOURCE_DIR}/marker_detector)
include_directories(${CMAKE_SOURCE_DIR}/pnp_solver)

add_executable(camera_calibration_tool camera_calibration_tool.cpp)
target_link_libraries(camera_calibration_tool ${OpenCV_LIBS} Eigen3::Eigen yaml-cpp calibration_model dir_common)
add_executable(sfm_tool sfm_tool.cpp)
target_link_libraries(sfm_tool sfm yaml-cpp dir_common sfm_simulation_data_generator)

add_executable(pose_viewer pose_viewer.cpp)
target_link_libraries(pose_viewer ${PCL_LIBRARIES} Eigen3::Eigen)

