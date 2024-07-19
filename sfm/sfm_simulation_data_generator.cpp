#include "sfm_simulation_data_generator.h"
#include "camera_model.h"
// MarkerData

namespace data_gen {
void SfmDataGenerator::generateData(
    std::vector<MarkerDetector::MarkerData> &marker_datas) {
  // 1.生成marker map
  std::vector<std::pair<int, std::vector<Eigen::Vector3d>>> marker_map;
  generateMarkerMap(marker_map);
  if (config_.debug) {
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
    pcl::io::savePCDFileASCII(config_.output_path + "marker_map.pcd",
                              *pointcloud);
  }
  // 2.生成pose序列
  std::vector<Eigen::Matrix4d> poses;
  //可视化可以用 pose_viewer的中工具查看相机视角和marker的关系
  generatePose(poses);
  // 3.生成 marker封装数据
  auto camera_model_ptr = CameraModelNS::CameraFactory::createCamera(
      config_.camera_type, config_.intrinsic_param, config_.resolution_w,
      config_.resolution_h, config_.distort_param);
  size_t counter = 0;
  for (const auto &pose : poses) {
    std::vector<std::pair<int, std::vector<Eigen::Vector2d>>> project_pt;
    projectMarkerMap(pose.inverse(), marker_map, camera_model_ptr, project_pt);
    marker_datas.push_back(MarkerDetector::MarkerData(project_pt));
    if (config_.debug) {
      if (!project_pt.empty()) {
        drawPointsOnImage(project_pt, camera_model_ptr->getReloX(),
                          camera_model_ptr->getReloY(),
                          config_.output_path + std::to_string(counter) +
                              ".png");
      } else {
        std::cout << "this image all out of range" << std::endl;
      }
      counter++;
    }
  }
}

void SfmDataGenerator::generateMarker3dPoint(
    std::vector<Eigen::Vector3d> &marker_3d_pt) {
  double coor_width = config_.marker_width * cos(M_PI / 4.0);
  marker_3d_pt.push_back(Eigen::Vector3d{0.0, coor_width, 0.0});
  marker_3d_pt.push_back(Eigen::Vector3d{coor_width, 0.0, 0.0});
  marker_3d_pt.push_back(Eigen::Vector3d{0.0, -1.0 * coor_width, 0.0});
  marker_3d_pt.push_back(Eigen::Vector3d{-1.0 * coor_width, 0.0, 0.0});
}

void SfmDataGenerator::generateMarkerPose(
    const std::vector<std::vector<double>> &pose_infos,
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

void SfmDataGenerator::generateMarkerMap(
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

void SfmDataGenerator::projectMarkerMap(
    const Eigen::Matrix4d &pose,
    const std::vector<std::pair<int, std::vector<Eigen::Vector3d>>> &marker_map,
    std::shared_ptr<CameraModelNS::CameraModel> camera_model,
    std::vector<std::pair<int, std::vector<Eigen::Vector2d>>> &project_pt) {
  for (const auto &marker : marker_map) {
    std::vector<Eigen::Vector2d> proj_pt;
    bool all_pt_on_img = true;
    for (const auto &pt : marker.second) {
      Eigen::Vector3d pt_in_cam =
          pose.block<3, 3>(0, 0) * pt + pose.block<3, 1>(0, 3);
      auto pt_img = camera_model->project(pt_in_cam);
      if (!camera_model->onImage(pt_img)) {
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
//从高处到低的s形状
// pose为:T_map_cam_n
void SfmDataGenerator::generatePose(std::vector<Eigen::Matrix4d> &poses) {
  for (size_t i = 0; i < config_.pose_num; ++i) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose(0, 3) = i * 0.5;
    pose(1, 3) = sin(i * 0.5);
    pose(2, 3) = cos(i * 0.5);
    // 随机生成四元数
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(i * 0.1, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(i * 0.1, Eigen::Vector3d::UnitZ());
    pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    poses.push_back(pose);
  }
}

void SfmDataGenerator::drawPointsOnImage(
    const std::vector<std::pair<int, std::vector<Eigen::Vector2d>>> &project_pt,
    int w, int h, const std::string &save_path) {
  // 创建白色背景图像
  cv::Mat image(h, w, CV_8UC3, cv::Scalar(255, 255, 255));
  // 遍历数据并绘制点
  for (const auto &pair : project_pt) {
    int label = pair.first;
    const std::vector<Eigen::Vector2d> &points = pair.second;
    // 获取颜色
    cv::Scalar color = colorMap[label % colorMap.size()];
    // 绘制每个点
    for (const auto &point : points) {
      cv::circle(image, cv::Point(point.x(), point.y()), 3, color, -1);
    }
  }
  // 显示图像
  cv::imwrite(save_path, image);
}
} // namespace data_gen