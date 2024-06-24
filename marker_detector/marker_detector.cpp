#include "marker_detector.h"

namespace MarkerDetector {

bool ApriltagMarkerDetector::detectMarker(
    cv::Mat img, MarkerData &output_marker_data) const {
  cv::Mat gray;
  cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};
  zarray_t *detections = apriltag_detector_detect(td_, &im);
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);
    std::vector<cv::Point2d> corners;
    corners.push_back(cv::Point2d(det->p[0][0], det->p[0][1]));
    corners.push_back(cv::Point2d(det->p[1][0], det->p[1][1]));
    corners.push_back(cv::Point2d(det->p[2][0], det->p[2][1]));
    corners.push_back(cv::Point2d(det->p[3][0], det->p[3][1]));
    output_marker_data.id_marker_corners.push_back(
        std::make_pair(det->id, corners));
  }
  return true;
}

} // namespace MarkerDetector