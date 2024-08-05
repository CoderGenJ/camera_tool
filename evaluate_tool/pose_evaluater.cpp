#include "pose_evaluater.h"
namespace evaluate_tool {

bool ComparePoseList(const std::vector<Eigen::Matrix4d> &pose_list_1,
                     const std::vector<Eigen::Matrix4d> &pose_list_2,
                     pcl::visualization::PCLVisualizer::Ptr &visualizer) {
  if (visualizer == nullptr) {
    return false;
  }
  visualizer->setBackgroundColor(0, 0, 0);
  // 添加全局坐标系
  visualizer->addCoordinateSystem(1.0); // 1.0表示坐标轴的长度
  auto add_pose_in_viewer =
      [](const std::vector<Eigen::Matrix4d> &pose_list,
         pcl::visualization::PCLVisualizer::Ptr &viewer) -> bool {
    if (viewer == nullptr) {
      return false;
    }
    for (size_t i = 0; i < pose_list.size(); ++i) {
      Eigen::Vector3d pose = pose_list.at(i).block<3, 1>(0, 3);
      Eigen::Quaterniond q{pose_list.at(i).block<3, 3>(0, 0)};
      // pose_list
      Eigen::Affine3d transform = Eigen::Affine3d::Identity();
      transform.translation() << pose.x(), pose.y(), pose.z();
      transform.rotate(q);

      std::string id = "pose_" + std::to_string(i);
      viewer->addCoordinateSystem(0.2, transform.cast<float>(),
                                  id); // 0.2表示每个pose坐标轴的长度
    }
    return true;
  };

  add_pose_in_viewer(pose_list_1, visualizer);
  add_pose_in_viewer(pose_list_2, visualizer);
  visualizer->initCameraParameters();
  visualizer->setCameraPosition(0, 0, 15, 0, -1, 0);
  // 循环直到视图关闭
  while (!visualizer->wasStopped()) {
    visualizer->spinOnce(100);
  }

  return true;
}

} // namespace evaluate_tool
