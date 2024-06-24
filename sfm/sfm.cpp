#include "sfm.h"
namespace SFM {

void structureFromMotion::extractMarker(cv::Mat img) {
  MarkerDetector::MarkerData marker_data;
  if (!marker_detector_ptr_->detectMarker(img, marker_data)) {
    return;
  }
}

} // namespace SFM