
include_directories(${CMAKE_SOURCE_DIR}/camera_model)
include_directories(${CMAKE_SOURCE_DIR}/factor)
include_directories(${CMAKE_SOURCE_DIR}/common)
add_library(sfm sfm.cpp)
target_link_libraries(sfm ${OpenCV_LIBRARIES} camera_model Eigen pnp_solver marker_detector)