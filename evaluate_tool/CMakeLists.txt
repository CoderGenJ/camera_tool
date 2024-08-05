
add_library(pose_evaluater  pose_evaluater.cpp)
target_link_libraries(pose_evaluater ${PCL_LIBRARIES} Eigen3::Eigen)