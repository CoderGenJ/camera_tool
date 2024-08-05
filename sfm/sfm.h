#pragma once
#include "camera_model.h"
#include "ceres_factor.hpp"
#include "marker_detector.h"
#include "pnp_solver.h"
// #include "pose_evaluater.h"
#include <Eigen/Dense>
#include <Eigen/Geometry> // 用于四元数
#include <functional>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <unordered_map>
#include <vector>
namespace SFM {
// marker 3d 坐标:
// 由于现实中marker的尺寸是知道的,所以,可以通过尺寸的边长来生成marker坐标系下的3D坐标
struct structureFromMotionConfig {
  structureFromMotionConfig() {}
  double marker_width_m = 0.5;
  size_t marker_corner_num = 4;
  std::string camera_type = "Pinhole";
  std::vector<double> intrin_param;
  std::vector<double> distorted_param;
  // width
  double reso_x;
  // height
  double reso_y;
  pnp_sovler::PnPSolverConfig pnp_config;
  MarkerDetector::ApriltagMarkerDetectorConfig apritag_config;
};

/// @brief marker在图像中保存的坐标
struct MarkerImgItem {
  MarkerImgItem() {}
  int marker_id;                                  // marker的编号
  std::vector<Eigen::Vector2d> marker_2ds;        //图像中的2D坐标
  std::vector<Eigen::Vector3d> marker_3ds_in_cam; //相机坐标系下的3D坐标
  // T_camera_marker
  data_common::Pose3d pose;
};
/// @brief 图片节点:包含图片的编号和marker的3d 2d坐标信息以及和各个board的转换
struct ImgNode {
  ImgNode() {}
  size_t index;
  // marker坐标
  std::vector<MarkerImgItem> marker_items;
  data_common::Pose3d T_map_current;
  data_common::Pose3d T_current_map;
};

/// @brief 从多张图片中恢复marker的3D坐标,生成对应的点云图
class structureFromMotion {
public:
  structureFromMotion(const structureFromMotionConfig &config)
      : config_(config) {
    camera_model_ptr_ = CameraModelNS::CameraFactory::createCamera(
        config_.camera_type, config_.intrin_param, config_.reso_x,
        config_.reso_y, config_.distorted_param);
    pnp_sovler_ptr_ = std::make_unique<pnp_sovler::PnPSolver>(
        config_.pnp_config, camera_model_ptr_);
    marker_detector_ptr_ =
        std::make_unique<MarkerDetector::ApriltagMarkerDetector>(
            config_.apritag_config);
    index_ = 0;
    find_marker_index_func_ =
        [](int marker_id, const std::vector<MarkerImgItem> &marker_iterms,
           size_t &index) -> bool {
      for (size_t i = 0; i < marker_iterms.size(); ++i) {
        if (marker_iterms.at(i).marker_id == marker_id) {
          index = i;
          return true;
        }
      }
      return false;
    };
    double coor_width = config_.marker_width_m * cos(M_PI / 4.0);
    board_corners_.push_back(cv::Point3d{0.0, coor_width, 0.0});
    board_corners_.push_back(cv::Point3d{coor_width, 0.0, 0.0});
    board_corners_.push_back(cv::Point3d{0.0, -1.0 * coor_width, 0.0});
    board_corners_.push_back(cv::Point3d{-1.0 * coor_width, 0.0, 0.0});
  }
  /// @brief 从图片中提取marker
  /// @param img
  bool detectMarker(cv::Mat img, MarkerDetector::MarkerData &output);

  /// @brief 将marker data插入到graph中
  void insertMarkerData(const MarkerDetector::MarkerData &marker_data);

  /// @brief 优化图像之间的pose graph地图,得出一系列pose
  /// @return
  bool optiPoseGraph();

  /// @brief 全局优化,map点和pose(T_map_current)
  /// @return
  bool fullBundleAdjustment();

  bool buildOptimizationProblem(ceres::Problem *problem);

  /// @brief 基于pose graph构建点云地图
  /// @return
  bool constructMap();

  /// @brief 全局优化,pose graph/map point
  bool buildBundleAdjustment(ceres::Problem *problem);

  bool comparePoseList(const std::vector<Eigen::Matrix4d> &pose_list_1,
                       const std::vector<Eigen::Matrix4d> &pose_list_2,
                       pcl::visualization::PCLVisualizer::Ptr &visualizer);
  bool displayPose(const std::vector<Eigen::Matrix4d> &true_poses);

private:
  structureFromMotionConfig config_;
  std::vector<cv::Point3d> board_corners_;
  std::unique_ptr<pnp_sovler::PnPSolver> pnp_sovler_ptr_;
  std::shared_ptr<CameraModelNS::CameraModel> camera_model_ptr_;
  std::unique_ptr<MarkerDetector::ApriltagMarkerDetector> marker_detector_ptr_;

  size_t index_;
  std::vector<ImgNode> img_nodes_;
  // marker id , img_index
  std::unordered_map<int, std::vector<size_t>> co_vis_;
  // marker id , map pt
  std::unordered_map<int, std::vector<Eigen::Vector3d>> map_;

  std::function<bool(int, const std::vector<MarkerImgItem> &, size_t &)>
      find_marker_index_func_;
};

} // namespace SFM