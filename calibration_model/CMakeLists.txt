set(CALIBRATION_MODEL_SOURCE calibration_model.cpp )
#  创建一个库或者可执行文件
add_library(calibration_model ${CALIBRATION_MODEL_SOURCE})
#  链接必要的库
target_link_libraries(calibration_model ${OpenCV_LIBS} Eigen3::Eigen)

# add_executable(camera_model_unit_test camera_model_unit_test.cpp)
# target_link_libraries(camera_model_unit_test ${GTEST_LIBRARIES} pthread camera_model)