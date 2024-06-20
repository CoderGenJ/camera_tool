add_library(marker_detector marker_detector.cpp)
target_link_libraries(marker_detector apriltag ${OpenCV_LIBRARIES})
