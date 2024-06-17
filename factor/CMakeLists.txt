include_directories(${CMAKE_SOURCE_DIR}/factor)
include_directories(${CMAKE_SOURCE_DIR}/camera_model)
include_directories(${CMAKE_SOURCE_DIR}/common)

add_executable(factor_unit_test factor_unit_test.cpp)
target_link_libraries(factor_unit_test ${GTEST_LIBRARIES} ${CERES_LIBRARIES} pthread camera_model)