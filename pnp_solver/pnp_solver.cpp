#include "pnp_solver.h"

namespace pnp_sovler {

bool PnPSolver::solvePnP(
    const std::vector<data_common::Point3d2dPair> &pt_3d_2d_pairs,
    Eigen::Matrix4d &output_rlt) {
  if (pt_3d_2d_pairs.size() < config_.min_size) {
    std::cout << "" << std::endl;
    return false;
  }
  //构建ceres
  ceres::Problem problem;
  ceres::LocalParameterization *quatParam =
      new ceres::QuaternionParameterization();
  // Define Loss function
  ceres::LossFunction *loss_function = new ceres::CauchyLoss(1);
  double R_camera_ref[4];
  double trans_camera_ref[3];
  R_camera_ref[0] = 1.0;
  R_camera_ref[1] = 0;
  R_camera_ref[2] = 0;
  R_camera_ref[3] = 0;
  trans_camera_ref[0] = 0.0;
  trans_camera_ref[1] = 0.0;
  trans_camera_ref[2] = 0.0;
  problem.AddParameterBlock(R_camera_ref, 4, quatParam);
  problem.AddParameterBlock(trans_camera_ref, 3);
  for (auto pair : pt_3d_2d_pairs) {
    problem.AddResidualBlock(
        ceres_factor::ReprojectErrorFactor::Create(camera_model_ptr_, pair),
        loss_function, R_camera_ref, trans_camera_ref);
  }
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.max_num_iterations = config_.ceres_max_iter;
  options.minimizer_progress_to_stdout =
      config_.ceres_minimizer_progress_to_stdout;
  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << std::endl;
  //优化结果
  Eigen::Quaterniond q_rlt{R_camera_ref[0], R_camera_ref[1], R_camera_ref[2],
                           R_camera_ref[3]};
  Eigen::Vector3d trans_rlt{trans_camera_ref[0], trans_camera_ref[1],
                            trans_camera_ref[2]};
  output_rlt = Eigen::Matrix4d::Identity();
  output_rlt.block<3, 3>(0, 0) = q_rlt.toRotationMatrix();
  output_rlt.block<3, 1>(0, 3) = trans_rlt;
  //结果检测
  double pixel_diff = 0.0;
  for (const auto &pt3d2d : pt_3d_2d_pairs) {
    Eigen::Vector3d pt3d_in_cam =
        output_rlt.block<3, 3>(0, 0) *
            Eigen::Vector3d{pt3d2d.pt3d.x, pt3d2d.pt3d.y, pt3d2d.pt3d.z} +
        output_rlt.block<3, 1>(0, 3);
    Eigen::Vector2d pt2d = camera_model_ptr_->project(pt3d_in_cam);
    double delta_x = pt3d2d.pt2d.x - pt2d.x();
    double delta_y = pt3d2d.pt2d.y - pt2d.y();
    pixel_diff += delta_x * delta_x + delta_y * delta_y;
  }
  double pixel_diff_average =
      pixel_diff / static_cast<double>(pt_3d_2d_pairs.size());
  if (pixel_diff_average > config_.pixel_diff * config_.pixel_diff) {
    std::cout << "average diff:" << pixel_diff_average
              << "more than:" << config_.pixel_diff * config_.pixel_diff
              << std::endl;
    return false;
  }
  return true;
}

} // namespace pnp_sovler