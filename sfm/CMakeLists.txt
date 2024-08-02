
include_directories(${CMAKE_SOURCE_DIR}/camera_model)
include_directories(${CMAKE_SOURCE_DIR}/factor)
include_directories(${CMAKE_SOURCE_DIR}/common)
include_directories(${CMAKE_SOURCE_DIR}/marker_detector)
include_directories(${CMAKE_SOURCE_DIR}/pnp_solver)
add_library(sfm sfm.cpp)
target_link_libraries(sfm ${OpenCV_LIBRARIES} camera_model pnp_solver marker_detector Eigen3::Eigen  ${PCL_LIBRARIES})

add_library(sfm_simulation_data_generator sfm_simulation_data_generator.cpp)
target_link_libraries(sfm_simulation_data_generator sfm yaml-cpp dir_common  ${PCL_LIBRARIES} camera_model)
