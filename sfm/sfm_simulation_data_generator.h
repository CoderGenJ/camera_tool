/*
    本工具用于生成SFM得仿真数据,主要为一个marker地图以及一系列相机姿态
*/

#include "camera_model.h"
#include "marker_detector.h"
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
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
  SfmDataGeneratorConfig() {}
  double marker_width = 0.5;
  bool debug = false;
  size_t pose_num = 20;
  // camear param
  //采用了TUM中的相机参数,不考虑畸变
  std::string camera_type = "Pinhole";
  std::vector<double> intrinsic_param{517.306408, 516.469215, 318.643040,
                                      255.313989};
  std::vector<double> distort_param{0.0, 0.0, 0.0, 0.0, 0.0};
  double resolution_w = 1920;
  double resolution_h = 1080;
  // output
  std::string output_path = "/home/eric/workspace/camera_tool/temp_data/";
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

  /// @brief 输出marker图片2D坐标以及pose
  ///  @param   output_marker_pose_ptr
  ///  输出图中有marker投影的图片对应的pose,T_map_cam
  void
  generateData(std::vector<MarkerDetector::MarkerData> &marker_datas,
               std::vector<Eigen::Matrix4d> *output_marker_pose_ptr = nullptr);

private:
  SfmDataGeneratorConfig config_;
};
} // namespace data_gen