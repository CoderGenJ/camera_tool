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
double marker_width = 0.5;
/// @brief 生成marker 3d 顺序为:左上右下,坐标系:右手坐标系,x轴向前,Z垂直纸面向外
/// @param marker_3d_pt
void generateMarker3dPoint(std::vector<Eigen::Vector3d> &marker_3d_pt);

/// @brief 生成marker得姿态
/// @param pose_info x,y,z,roll,pitch,yaw
/// @param marker_pose
void generateMarkerPose(const std::vector<std::vector<double>> &pose_infos,
                        std::vector<Eigen::Matrix4d> &marker_pose);

void generateMarkerMap(
    std::vector<std::pair<int, std::vector<Eigen::Vector3d>>> &marker_map);

void generatePose(std::vector<Eigen::Matrix4d> &poses);
void projectMarkerMap(
    const Eigen::Matrix4d &pose,
    const std::vector<std::pair<int, std::vector<Eigen::Vector3d>>> &marker_map,
    const CameraModelNS::CameraModel camera_model,
    std::vector<std::pair<int, std::vector<Eigen::Vector2d>>> project_pt);

int main() {
  std::vector<std::pair<int, std::vector<Eigen::Vector3d>>> marker_map;
  generateMarkerMap(marker_map);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto &marker : marker_map) {
    const auto &pts = marker.second;
    for (const auto &pt : pts) {
      pcl::PointXYZ pt_pd;
      pt_pd.x = pt.x();
      pt_pd.y = pt.y();
      pt_pd.z = pt.z();
      pointcloud->points.push_back(pt_pd);
    }
  }
  pointcloud->width = 1;
  pointcloud->height = pointcloud->points.size();
  pcl::io::savePCDFileASCII("/home/eric/marker_map.pcd", *pointcloud);

  return 0;
}

void generateMarker3dPoint(std::vector<Eigen::Vector3d> &marker_3d_pt) {
  double coor_width = marker_width * cos(M_PI / 4.0);
  marker_3d_pt.push_back(Eigen::Vector3d{0.0, coor_width, 0.0});
  marker_3d_pt.push_back(Eigen::Vector3d{coor_width, 0.0, 0.0});
  marker_3d_pt.push_back(Eigen::Vector3d{0.0, -1.0 * coor_width, 0.0});
  marker_3d_pt.push_back(Eigen::Vector3d{-1.0 * coor_width, 0.0, 0.0});
}

void generateMarkerPose(const std::vector<std::vector<double>> &pose_infos,
                        std::vector<Eigen::Matrix4d> &marker_pose) {
  for (const auto &pose_info : pose_infos) {
    Eigen::Matrix4d pose(Eigen::Matrix4d::Identity());
    Eigen::AngleAxisd rotationX(pose_info[3], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rotationY(pose_info[4], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rotationZ(pose_info[5], Eigen::Vector3d::UnitZ());
    pose.block<3, 3>(0, 0) =
        (rotationX * rotationY * rotationZ).toRotationMatrix();
    pose.block<3, 1>(0, 3) =
        Eigen::Vector3d{pose_info[0], pose_info[1], pose_info[2]};
    marker_pose.push_back(pose);
  }
}

void generateMarkerMap(
    std::vector<std::pair<int, std::vector<Eigen::Vector3d>>> &marker_map) {
  std::vector<std::vector<double>> pose_infos;
  pose_infos.push_back(
      std::vector<double>{10.0, 0, 3.0, 0.0, -1.0 * M_PI / 2.0, 0.0});
  pose_infos.push_back(std::vector<double>{9.0, -2.0, 3.0, -1.0 * M_PI / 6.0,
                                           -1.0 * M_PI / 2.0, 0.0});
  pose_infos.push_back(std::vector<double>{9.0, 2.0, 3.0, 1.0 * M_PI / 6.0,
                                           -1.0 * M_PI / 2.0, 0.0});
  pose_infos.push_back(std::vector<double>{8.0, -4.0, 1.5, -1.0 * M_PI / 5.0,
                                           -1.0 * M_PI / 2.0, 0.0});
  pose_infos.push_back(std::vector<double>{8.0, 4.0, 1.5, 1.0 * M_PI / 5.0,
                                           -1.0 * M_PI / 2.0, 0.0});
  pose_infos.push_back(std::vector<double>{7.0, 6.0, 1.0, 1.0 * M_PI / 5.0,
                                           -1.0 * M_PI / 2.0, 0.0});
  pose_infos.push_back(std::vector<double>{7.0, -6.0, 1.0, 1.0 * M_PI / 5.0,
                                           -1.0 * M_PI / 2.0, 0.0});
  pose_infos.push_back(std::vector<double>{
      6.0, 5.0, 1.0, 1.0 * M_PI / 5.0, -1.0 * M_PI / 2.0, 1.0 * M_PI / 2.0});
  pose_infos.push_back(std::vector<double>{
      6.0, -5.0, 1.0, 1.0 * M_PI / 5.0, -1.0 * M_PI / 2.0, 1.0 * M_PI / 2.0});
  std::vector<Eigen::Matrix4d> marker_poses;
  generateMarkerPose(pose_infos, marker_poses);
  std::vector<Eigen::Vector3d> marker_3d_pt;
  generateMarker3dPoint(marker_3d_pt);
  size_t counter = 0;
  for (const auto &pose : marker_poses) {
    std::vector<Eigen::Vector3d> marker_pts;
    for (const auto pt : marker_3d_pt) {
      marker_pts.push_back(pose.block<3, 3>(0, 0) * pt +
                           pose.block<3, 1>(0, 3));
    }
    marker_map.push_back(std::make_pair(counter, marker_pts));
    counter++;
  }
}

void projectMarkerMap(
    const Eigen::Matrix4d &pose,
    const std::vector<std::pair<int, std::vector<Eigen::Vector3d>>> &marker_map,
    const CameraModelNS::CameraModel camera_model,
    std::vector<std::pair<int, std::vector<Eigen::Vector2d>>> project_pt) {
  for (const auto &marker : marker_map) {
    std::vector<Eigen::Vector2d> proj_pt;
    bool all_pt_on_img = true;
    for (const auto &pt : marker.second) {
      Eigen::Vector3d pt_in_cam =
          pose.block<3, 3>(0, 0) * pt + pose.block<3, 1>(0, 3);
      auto pt_img = camera_model.project(pt_in_cam);
      if (!camera_model.onImage(pt_img)) {
        all_pt_on_img = false;
        break;
      }
      proj_pt.push_back(pt_img);
    }
    if (all_pt_on_img) {
      project_pt.push_back(std::make_pair(marker.first, proj_pt));
    }
  }
}
//向前,走直线
void generatePose(std::vector<Eigen::Matrix4d> &poses) {
  

}