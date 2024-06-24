#include "camera_model.h"
#include "ceres_factor.hpp"
#include "marker_detector.h"
#include "pnp_solver.h"
#include <opencv2/opencv.hpp>
#include <vector>
namespace SFM {
// marker 3d 坐标:
// 由于现实中marker的尺寸是知道的,所以,可以通过尺寸的边长来生成marker坐标系下的3D坐标
struct structureFromMotionConfig {
  double marker_width_m = 0.5;
  std::string camera_type = "Pinhole";
  std::vector<double> intrin_param;
  double reso_x;
  double reso_y;
  std::vector<double> distorted_param;
  pnp_sovler::PnPSolverConfig pnp_config;
  MarkerDetector::ApriltagMarkerDetectorConfig apritag_config;
};
struct ImgProperty {
  size_t index;
  // index,markerdata(marker_id,2d in img)
  std::vector<std::pair<size_t, MarkerDetector::MarkerData>> imgs_markers;
  // index,Transform:T_self_other
  std::vector<std::pair<size_t, Eigen::Matrix4d>> pose_edges;
  // marker id,mark 3d in camera,T_camera_marker
  std::vector<std::tuple<size_t, std::vector<Eigen::Vector3d>,
                         std::vector<Eigen::Matrix4d>>>
      maker_info;
};
/// @brief 从多张图片中恢复marker的3D坐标,生成对应的点云图
class structureFromMotion {
public:
  structureFromMotion(const structureFromMotionConfig &config)
      : config_(config) {
    std::shared_ptr<CameraModelNS::CameraModel> camera_model_ptr =
        CameraModelNS::CameraFactory::createCamera(
            config_.camera_type, config_.intrin_param, config_.reso_x,
            config_.reso_y, config_.distorted_param);
    pnp_sovler_ptr_ = std::make_unique<pnp_sovler::PnPSolver>(
        config_.pnp_config, camera_model_ptr);
    marker_detector_ptr_ =
        std::make_unique<MarkerDetector::ApriltagMarkerDetector>(
            config_.apritag_config);
    index_ = 0;
  }
  /// @brief 从图片中提取marker
  /// @param img
  void extractMarker(cv::Mat img);


private:
  structureFromMotionConfig config_;
  std::vector<Eigen::Vector3d> board_corners_;
  std::unique_ptr<pnp_sovler::PnPSolver> pnp_sovler_ptr_;
  std::unique_ptr<MarkerDetector::ApriltagMarkerDetector> marker_detector_ptr_;
  size_t index_;

  std::vector<ImgProperty> imgs_propertys_;
};

} // namespace SFM