#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry> // 用于四元数
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

namespace evaluate_tool {

bool ComparePoseList(const std::vector<Eigen::Matrix4d> &pose_list_1,
                     const std::vector<Eigen::Matrix4d> &pose_list_2,
                     pcl::visualization::PCLVisualizer::Ptr &visualizer);

};