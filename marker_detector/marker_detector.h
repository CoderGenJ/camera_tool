#include <iomanip>
#include <iostream>

#include "opencv2/opencv.hpp"

extern "C" {
#include "apriltag.h"
#include "common/getopt.h"
#include "tag16h5.h"
#include "tag25h9.h"
#include "tag36h11.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
}
namespace MarkerDetector {

struct MarkerData {
  MarkerData() {}
  // marker id , marker_2d in image
  std::vector<std::pair<int, std::vector<cv::Point2d>>> id_marker_corners;
};

class BaseMarkerDetector {
public:
  BaseMarkerDetector() {}
  /// @brief 检测图像的marker并输出id和坐标
  /// @param img
  /// @param output_marker_data
  /// @return
  virtual bool detectMarker(cv::Mat img,
                            MarkerData &output_marker_data) const = 0;
  virtual ~BaseMarkerDetector() {}
};

struct ApriltagMarkerDetectorConfig {
  ApriltagMarkerDetectorConfig() {}
  ApriltagMarkerDetectorConfig(const std::string &_family, float _quad_decimate,
                               float _quad_sigma, int _nthreads, bool _debug,
                               bool _refine_edges)
      : family(_family), quad_decimate(_quad_decimate), quad_sigma(_quad_sigma),
        nthreads(_nthreads), debug(_debug), refine_edges(_refine_edges) {}
  std::string family;
  float quad_decimate;
  float quad_sigma;
  int nthreads=1;
  bool debug;
  bool refine_edges;
};

/// @brief 针对apriltag marker的检测
class ApriltagMarkerDetector : public BaseMarkerDetector {
public:
  ApriltagMarkerDetector(const ApriltagMarkerDetectorConfig &config)
      : config_(config) {
    tf_ = NULL;
    const char *famname = config_.family.c_str();
    if (!strcmp(famname, "tag36h11")) {
      tf_ = tag36h11_create();
    } else if (!strcmp(famname, "tag25h9")) {
      tf_ = tag25h9_create();
    } else if (!strcmp(famname, "tag16h5")) {
      tf_ = tag16h5_create();
    } else if (!strcmp(famname, "tagCircle21h7")) {
      tf_ = tagCircle21h7_create();
    } else if (!strcmp(famname, "tagCircle49h12")) {
      tf_ = tagCircle49h12_create();
    } else if (!strcmp(famname, "tagStandard41h12")) {
      tf_ = tagStandard41h12_create();
    } else if (!strcmp(famname, "tagStandard52h13")) {
      tf_ = tagStandard52h13_create();
    } else if (!strcmp(famname, "tagCustom48h12")) {
      tf_ = tagCustom48h12_create();
    } else {
      printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
    }
    apriltag_detector_add_family(td_, tf_);

    td_->quad_decimate = config_.quad_decimate;
    td_->quad_sigma = config_.quad_sigma;
    td_->nthreads = config_.nthreads;
    td_->debug = config_.debug;
    td_->refine_edges = config_.refine_edges;
  }
  bool detectMarker(cv::Mat img, MarkerData &output_marker_data) const override;
  ~ApriltagMarkerDetector() {
    apriltag_detector_destroy(td_);
    const char *famname = config_.family.c_str();
    if (!strcmp(famname, "tag36h11")) {
      tag36h11_destroy(tf_);
    } else if (!strcmp(famname, "tag25h9")) {
      tag25h9_destroy(tf_);
    } else if (!strcmp(famname, "tag16h5")) {
      tag16h5_destroy(tf_);
    } else if (!strcmp(famname, "tagCircle21h7")) {
      tagCircle21h7_destroy(tf_);
    } else if (!strcmp(famname, "tagCircle49h12")) {
      tagCircle49h12_destroy(tf_);
    } else if (!strcmp(famname, "tagStandard41h12")) {
      tagStandard41h12_destroy(tf_);
    } else if (!strcmp(famname, "tagStandard52h13")) {
      tagStandard52h13_destroy(tf_);
    } else if (!strcmp(famname, "tagCustom48h12")) {
      tagCustom48h12_destroy(tf_);
    }
  }

private:
  ApriltagMarkerDetectorConfig config_;
  apriltag_detector_t *td_;
  apriltag_family_t *tf_;
};

} // namespace MarkerDetector
