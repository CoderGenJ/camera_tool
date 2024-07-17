#include <Eigen/Geometry> // 用于四元数
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv) {
  // 创建PCL可视化对象
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Trajectory Viewer"));

  // 设置背景色
  viewer->setBackgroundColor(0, 0, 0);

  // 添加全局坐标系
  viewer->addCoordinateSystem(1.0); // 1.0表示坐标轴的长度

  // 定义点云类型
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // 假设轨迹点云数据已经加载到cloud中，并且有相应的方向数据
  // 这里我们生成一些示例数据
  cloud->width = 10;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  std::vector<Eigen::Quaternionf> orientations;
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x = i * 0.5;
    cloud->points[i].y = sin(i * 0.5);
    cloud->points[i].z = cos(i * 0.5);

    // 随机生成四元数
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(i * 0.1, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(i * 0.1, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(i * 0.1, Eigen::Vector3f::UnitZ());
    orientations.push_back(q);
  }

  // 将轨迹点云添加到视图中
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "trajectory");

  // 添加每个pose的坐标系
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    pcl::PointXYZ point = cloud->points[i];
    Eigen::Quaternionf q = orientations[i];

    // 创建变换矩阵
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << point.x, point.y, point.z;
    transform.rotate(q);

    std::string id = "pose_" + std::to_string(i);
    viewer->addCoordinateSystem(0.2, transform,
                                id); // 0.2表示每个pose坐标轴的长度
  }

  // 设置相机位置和角度
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 15, 0, -1, 0);

  // 循环直到视图关闭
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  }

  return 0;
}
