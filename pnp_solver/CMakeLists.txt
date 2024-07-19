
include_directories(${CMAKE_SOURCE_DIR}/camera_model)
include_directories(${CMAKE_SOURCE_DIR}/factor)
include_directories(${CMAKE_SOURCE_DIR}/common)

add_library(pnp_solver pnp_solver.cpp)
set_target_properties(pnp_solver PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(pnp_solver ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

add_executable(pnp_solver_unit_test pnp_solver_unit_test.cpp)
target_link_libraries(pnp_solver_unit_test pnp_solver ${GTEST_LIBRARIES} camera_model)

