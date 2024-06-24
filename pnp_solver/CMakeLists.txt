
include_directories(${CMAKE_SOURCE_DIR}/camera_model)
include_directories(${CMAKE_SOURCE_DIR}/factor)
include_directories(${CMAKE_SOURCE_DIR}/common)
add_library(pnp_solver pnp_solver.cpp)
target_link_libraries(pnp_solver ${OpenCV_LIBRARIES} camera_model)