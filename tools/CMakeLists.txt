# 创建一个库或者可执行文件
 add_executable(calibration_tool calibration_tool.cpp)
# 链接必要的库
target_include_directories(calibration_tool
    PRIVATE
        ${CMAKE_SOURCE_DIR}/calibration_model
        ${CMAKE_SOURCE_DIR}/common
)
target_link_libraries(calibration_tool ${OpenCV_LIBS} Eigen3::Eigen calibration_model dir_common) 
add_dependencies(calibration_tool calibration_model dir_common)
