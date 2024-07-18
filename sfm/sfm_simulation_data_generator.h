/*
    本工具用于生成SFM得仿真数据,主要为一个marker地图以及一系列相机姿态
*/

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

#include "camera_model.h"
// MarkerData
std::map<int, cv::Scalar> colorMap = {
    {0, cv::Scalar(255, 0, 0)},   // 红色
    {1, cv::Scalar(0, 255, 0)},   // 绿色
    {2, cv::Scalar(0, 0, 255)},   // 蓝色
    {3, cv::Scalar(255, 255, 0)}, // 青色
    {4, cv::Scalar(255, 0, 255)}, // 洋红色
    {5, cv::Scalar(0, 255, 255)}  // 黄色
};
namespace data_gen {
struct SfmDataGeneratorConfig {
  double marker_width = 0.5;
};

class SfmDataGenerator {
public:
  SfmDataGenerator(const SfmDataGeneratorConfig &config) : config_(config) {}
  /// @brief 生成marker 3d
  /// 顺序为:左上右下,坐标系:右手坐标系,x轴向前,Z垂直纸面向外
  /// @param marker_3d_pt
  void generateMarker3dPoint(std::vector<Eigen::Vector3d> &marker_3d_pt);

  /// @brief 生成marker得姿态
  /// @param pose_info x,y,z,roll,pitch,yaw
  /// @param marker_pose
  void generateMarkerPose(const std::vector<std::vector<double>> &pose_infos,
                          std::vector<Eigen::Matrix4d> &marker_pose);

  void generateMarkerMap(
      std::vector<std::pair<int, std::vector<Eigen::Vector3d>>> &marker_map);
  // 从高处到低的s形状
  // pose为:T_map_cam_n
  void generatePose(std::vector<Eigen::Matrix4d> &poses);
  void projectMarkerMap(
      const Eigen::Matrix4d &pose,
      const std::vector<std::pair<int, std::vector<Eigen::Vector3d>>>
          &marker_map,
      std::shared_ptr<CameraModelNS::CameraModel> camera_model,
      std::vector<std::pair<int, std::vector<Eigen::Vector2d>>> &project_pt);
  void drawPointsOnImage(
      const std::vector<std::pair<int, std::vector<Eigen::Vector2d>>>
          &project_pt,
      int w, int h, const std::string &save_path);
  //生成marker数据
  void generateData();

private:
  SfmDataGeneratorConfig config_;
};
} // namespace data_gen