#  设置子目录的源文件
set(TOOLS_SOURCES camera_model.cpp )
#  创建一个库或者可执行文件
add_library(camera_model ${TOOLS_SOURCES})
#  链接必要的库
target_link_libraries(camera_model ${OpenCV_LIBS} Eigen3::Eigen)