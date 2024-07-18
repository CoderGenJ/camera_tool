#include "sfm.h"
namespace SFM {

bool structureFromMotion::detectMarker(cv::Mat img,
                                       MarkerDetector::MarkerData &output) {
  if (!marker_detector_ptr_->detectMarker(img, output)) {
    return false;
  }
  return true;
}

void structureFromMotion::insertMarkerData(
    const MarkerDetector::MarkerData &marker_data) {
  //初始化图片编号
  ImgNode img_node;
  img_node.index = index_;
  index_++;
  // 遍历提取marker的信息
  for (const auto &marker_item : marker_data.id_marker_corners) {
    auto id = marker_item.first;
    auto marker_2ds = marker_item.second;
    std::vector<data_common::Point3d2dPair> pairs;
    for (size_t i = 0; i < marker_2ds.size(); ++i) {
      pairs.push_back(
          data_common::Point3d2dPair(marker_2ds.at(i), board_corners_.at(i)));
    }
    //将新的marker保存到序列中
    if (co_vis_.find(id) == co_vis_.end()) {
      co_vis_.insert(std::make_pair(id, std::vector<size_t>()));
    }
    co_vis_[id].push_back(img_node.index);

    Eigen::Matrix4d T_cam_board;
    if (!pnp_sovler_ptr_->solvePnP(pairs, T_cam_board)) {
      continue;
    }
    MarkerImgItem marker_img_item;
    for (size_t j = 0; j < board_corners_.size(); ++j) {
      auto &board_corner = board_corners_.at(j);
      Eigen::Vector3d pt3d_in_cam =
          T_cam_board.block<3, 3>(0, 0) *
              Eigen::Vector3d{board_corner.x, board_corner.y, board_corner.z} +
          T_cam_board.block<3, 1>(0, 3);
      marker_img_item.marker_3ds_in_cam.push_back(pt3d_in_cam);
      marker_img_item.marker_id = id;
      marker_img_item.marker_2ds.push_back(
          Eigen::Vector2d{marker_2ds.at(j).x, marker_2ds.at(j).y});
      marker_img_item.pose = data_common::Pose3d(T_cam_board);
    }

    img_node.marker_items.push_back(marker_img_item);
  }
  //将marker的节点信息保存
  img_nodes_.push_back(img_node);
}

bool structureFromMotion::constructMap() {
  if (co_vis_.empty()) {
    return false;
  }
  // 1.遍历所有marker id
  for (auto co_vis_iter = co_vis_.begin(); co_vis_iter != co_vis_.end();
       ++co_vis_iter) {
    std::vector<Eigen::Vector3d> marker_average_map(config_.marker_corner_num);
    auto marker_id = co_vis_iter->first;
    for (size_t i = 0; i < co_vis_iter->second.size(); ++i) {
      auto &node = img_nodes_.at(i);
      const auto &marker_iterm = node.marker_items;
      size_t index;
      if (!find_marker_index_func_(marker_id, marker_iterm, index)) {
        continue;
      }
      auto &marker3d_in_cam = marker_iterm.at(index).marker_3ds_in_cam;
      if (marker_iterm.at(index).marker_3ds_in_cam.size() !=
          config_.marker_corner_num) {
        std::cout << "marker数不一样" << std::endl;
        return false;
      }
      for (size_t i = 0; i < marker3d_in_cam.size(); ++i) {
        auto mk3d_in_map = node.T_map_current.transfPt(marker3d_in_cam.at(i));
        marker_average_map.at(i) += mk3d_in_map;
      }
    }
    for (size_t i = 0; i < marker_average_map.size(); ++i) {
      marker_average_map.at(i) =
          marker_average_map.at(i) /
          static_cast<double>(co_vis_iter->second.size());
    }
    map_.insert(std::make_pair(marker_id, marker_average_map));
  }
  return true;
}

bool structureFromMotion::buildOptimizationProblem(ceres::Problem *problem) {
  if (img_nodes_.empty()) {
    return false;
  }
  ceres::LossFunction *loss_function = NULL;
  ceres::LocalParameterization *quaternion_local_parameterization =
      new ceres::EigenQuaternionParameterization;
  const Eigen::Matrix<double, 6, 6> sqrt_information =
      Eigen::Matrix<double, 6, 6>::Identity().llt().matrixL();

  for (auto co_vis_iter = co_vis_.begin(); co_vis_iter != co_vis_.end();
       ++co_vis_iter) {
    auto marker_id = co_vis_iter->first;
    for (size_t i = 0; i < co_vis_iter->second.size(); ++i) {
      auto &a_node = img_nodes_.at(i);
      const auto &a_marker_iterm = a_node.marker_items;
      size_t a_index;
      if (!find_marker_index_func_(marker_id, a_marker_iterm, a_index)) {
        continue;
      }
      const auto &T_a_board = a_marker_iterm.at(a_index).pose;
      for (size_t j = i + 1; j < co_vis_iter->second.size(); ++j) {
        auto &b_node = img_nodes_.at(j);
        const auto &b_marker_iterm = a_node.marker_items;
        size_t b_index;
        if (!find_marker_index_func_(marker_id, b_marker_iterm, b_index)) {
          continue;
        }
        const auto &T_b_board = b_marker_iterm.at(b_index).pose;
        auto T_board_b = T_b_board.inverse();
        auto T_a_b_mearured = T_a_board * T_board_b;
        ceres::CostFunction *cost_function =
            ceres_factor::Pose3dErrorFactor::Create(T_a_b_mearured,
                                                    sqrt_information);
        problem->AddResidualBlock(cost_function, loss_function,
                                  a_node.T_map_current.p.data(),
                                  a_node.T_map_current.q.coeffs().data(),
                                  b_node.T_map_current.p.data(),
                                  b_node.T_map_current.q.coeffs().data());
        problem->SetParameterization(a_node.T_map_current.q.coeffs().data(),
                                     quaternion_local_parameterization);
        problem->SetParameterization(b_node.T_map_current.q.coeffs().data(),
                                     quaternion_local_parameterization);
      }
    }
  }
  //固定第一帧
  problem->SetParameterBlockConstant(img_nodes_[0].T_map_current.p.data());
  problem->SetParameterBlockConstant(
      img_nodes_[0].T_map_current.q.coeffs().data());
  return true;
}

bool structureFromMotion::optiPoseGraph() {
  ceres::Problem problem;
  buildOptimizationProblem(&problem);
  ceres::Solver::Options options;
  options.max_num_iterations = 200;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << '\n';
  return summary.IsSolutionUsable();
}

bool structureFromMotion::buildBundleAdjustment(ceres::Problem *problem) {
  ceres::LossFunction *loss_function = NULL;
  ceres::LocalParameterization *quaternion_local_parameterization =
      new ceres::EigenQuaternionParameterization;
  // 1.遍历每一个图片
  for (size_t i = 0; i < img_nodes_.size(); ++i) {
    auto &node = img_nodes_.at(i);
    //求解的是T_current_map,但是保存的T_map_current,为保证初始值,需要求逆
    node.T_current_map = node.T_map_current.inverse();
    // 2.遍历每一个board marker
    for (const auto &marker_item : node.marker_items) {
      // 3.遍历每个corner
      for (size_t j = 0; j < marker_item.marker_2ds.size(); ++j) {
        ceres::CostFunction *cost_function =
            ceres_factor::ReprojectErrorMapPoseFactor::Create(
                camera_model_ptr_, marker_item.marker_2ds.at(j));
        problem->AddResidualBlock(cost_function, loss_function,
                                  node.T_current_map.p.data(),
                                  node.T_current_map.q.coeffs().data(),
                                  map_[marker_item.marker_id].at(j).data());
        problem->SetParameterization(node.T_current_map.q.coeffs().data(),
                                     quaternion_local_parameterization);
      }
    }
  }

  return true;
}

bool structureFromMotion::fullBundleAdjustment() {
  ceres::Problem problem;
  buildBundleAdjustment(&problem);
  ceres::Solver::Options options;
  options.max_num_iterations = 200;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << '\n';
  return summary.IsSolutionUsable();
}

} // namespace SFM