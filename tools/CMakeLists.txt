# 创建一个库或者可执行文件
 add_executable(pinholeCalibrationTool pinhole_calibration_tool.cpp)
# 链接必要的库
target_link_libraries(pinholeCalibrationTool ${OpenCV_LIBS} Eigen3::Eigen)