#include "pnp_solver.h"

namespace pnp_sovler {

bool PnPSolver::solvePnP(
    const std::vector<data_common::Point3d2dPair> &pt_3d_2d_pairs,
    Eigen::Matrix4d &inoutput_rlt) {
  if (pt_3d_2d_pairs.size() < config_.min_size) {
    std::cout << "pnp failed,the pt pairs < " << config_.min_size << std::endl;
    return false;
  }
  // 1.处理初值
  Eigen::Quaterniond q_init(inoutput_rlt.block<3, 3>(0, 0));
  double R_camera_ref[4];
  double trans_camera_ref[3];
  R_camera_ref[0] = q_init.w();
  R_camera_ref[1] = q_init.x();
  R_camera_ref[2] = q_init.y();
  R_camera_ref[3] = q_init.z();
  trans_camera_ref[0] = inoutput_rlt(0, 3);
  trans_camera_ref[1] = inoutput_rlt(1, 3);
  trans_camera_ref[2] = inoutput_rlt(2, 3);
  // 2.构建ceres求解
  //构建ceres
  ceres::Problem problem;
  ceres::LocalParameterization *quatParam =
      new ceres::QuaternionParameterization();
  // Define Loss function
  ceres::LossFunction *loss_function = new ceres::CauchyLoss(1);
  problem.AddParameterBlock(R_camera_ref, 4, quatParam);
  problem.AddParameterBlock(trans_camera_ref, 3);
  for (auto pair : pt_3d_2d_pairs) {
    problem.AddResidualBlock(
        ceres_factor::ReprojectErrorFactor::Create(camera_model_ptr_, pair),
        nullptr, R_camera_ref, trans_camera_ref);
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
  inoutput_rlt = Eigen::Matrix4d::Identity();
  inoutput_rlt.block<3, 3>(0, 0) = q_rlt.toRotationMatrix();
  inoutput_rlt.block<3, 1>(0, 3) = trans_rlt;
  //结果检测
  double pixel_diff = 0.0;
  for (const auto &pt3d2d : pt_3d_2d_pairs) {
    Eigen::Vector3d pt3d_in_cam =
        inoutput_rlt.block<3, 3>(0, 0) *
            Eigen::Vector3d{pt3d2d.pt3d.x, pt3d2d.pt3d.y, pt3d2d.pt3d.z} +
        inoutput_rlt.block<3, 1>(0, 3);
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

bool PnPSolver::solvePnPOpencv(
    const std::vector<data_common::Point3d2dPair> &pt_3d_2d_pairs,
    Eigen::Matrix4d &inoutput_rlt) {
  //构建opencv的3d 2d输入对
  std::vector<cv::Point3d> objectPoints;
  std::vector<cv::Point2d> imagePoints;
  for (const auto &pair : pt_3d_2d_pairs) {
    objectPoints.push_back(pair.pt3d);
    imagePoints.push_back(pair.pt2d);
  }
  //构建相机矩阵,假设无畸变
  auto intrinsic_param = camera_model_ptr_->getIntrinsicParam();
  cv::Mat cameraMatrix =
      (cv::Mat_<double>(3, 3) << intrinsic_param[0], 0, intrinsic_param[2], 0,
       intrinsic_param[1], intrinsic_param[3], 0, 0, 1);
  cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
  // pnp求解
  cv::Mat rvec, tvec;
  bool success = cv::solvePnP(objectPoints, imagePoints, cameraMatrix,
                              distCoeffs, rvec, tvec, false, cv::SOLVEPNP_EPNP);
  // 检查算法是否成功
  if (success) {
    cv::Mat rotationMatrix;
    cv::Rodrigues(rvec, rotationMatrix);
    // 将 OpenCV 的旋转矩阵和平移向量转换为 Eigen 矩阵
    Eigen::Matrix3d rotationMatrixEigen;
    Eigen::Vector3d translationVectorEigen;

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        rotationMatrixEigen(i, j) = rotationMatrix.at<double>(i, j);
      }
    }
    for (int i = 0; i < 3; ++i) {
      translationVectorEigen(i) = tvec.at<double>(i, 0);
    }
    inoutput_rlt.block<3, 3>(0, 0) = rotationMatrixEigen;
    inoutput_rlt.block<3, 1>(0, 3) = translationVectorEigen;
  } else {
    std::cout << "EPnP 算法未能找到解决方案。" << std::endl;
  }
  return success;
}

} // namespace pnp_sovler