#include <ceres/ceres.h>
#include <gtest/gtest.h>

#include "camera_model.h"
#include "ceres_factor.hpp"
#include "transform_common.hpp"

void generateBoardPoint(std::vector<Eigen::Vector3d> &output_pd,
                        double marker_width);

TEST(CERES_FACTOR, CERES_FACTOR) {
  //生成3D点
  std::vector<Eigen::Vector3d> pts_3d;
  generateBoardPoint(pts_3d, 1.0);
  //生成相机模型
  std::vector<double> intrinsic_param{800, 800, 640, 480};
  std::vector<double> distorted_param{0.0, 0.0, 0.0, 0.0, 0.0};
  std::shared_ptr<CameraModelNS::PinholeCameraModel> cam_model(
      new CameraModelNS::PinholeCameraModel(intrinsic_param, distorted_param,
                                            1000, 2000));
  //生成生成转换
  Eigen::Matrix3d rotation =
      transform_common::eulerAngleToMatrix<double>(1.0, 1.1, 1.2);
  Eigen::Vector3d trans{1.2, 1.3, 1.4};
  std::vector<data_common::Point3d2dPair> pairs;
  //生成3D-2D匹配对
  for (const auto &pt3d : pts_3d) {
    Eigen::Vector3d pt3d_in_cam = rotation * pt3d + trans;
    Eigen::Vector2d pt2d = cam_model->project(pt3d_in_cam);
    pairs.push_back(
        data_common::Point3d2dPair(cv::Point2d(pt2d.x(), pt2d.y()),
                                   cv::Point3d(pt3d.x(), pt3d.y(), pt3d.z())));
  }
  //构建ceres
  ceres::Problem problem;
  ceres::LocalParameterization *quatParam =
      new ceres::QuaternionParameterization();
  // Define Loss function
  ceres::LossFunction *loss_function = new ceres::CauchyLoss(0.5);

  Eigen::Quaterniond quat =
      transform_common::eulerAngleToQuat<double>(0.9, 1.0, 1.1);
  double R_camera_ref[4];
  double trans_camera_ref[3];
  R_camera_ref[0] = quat.w();
  R_camera_ref[1] = quat.x();
  R_camera_ref[2] = quat.y();
  R_camera_ref[3] = quat.z();

  trans_camera_ref[0] = 1.0;
  trans_camera_ref[1] = 1.0;
  trans_camera_ref[2] = 1.2;
  problem.AddParameterBlock(R_camera_ref, 4, quatParam);
  problem.AddParameterBlock(trans_camera_ref, 3);

  for (auto pair : pairs) {
    problem.AddResidualBlock(
        ceres_factor::Camera3D2DFactor::Create(cam_model, pair), loss_function,
        R_camera_ref, trans_camera_ref);
  }
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.max_num_iterations = 30;
  options.minimizer_progress_to_stdout = true;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << std::endl;
  //优化结果
  Eigen::Map<Eigen::Quaterniond> q_rlt(R_camera_ref);
  Eigen::Map<Eigen::Vector3d> t_rlt(trans_camera_ref);
  //对比优化结果
  std::cout << "euler:"
            << q_rlt.toRotationMatrix().eulerAngles(2, 1, 0).transpose();
  std::cout << "trans:" << t_rlt.transpose();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

void generateBoardPoint(std::vector<Eigen::Vector3d> &output_pd,
                        double marker_width) {
  marker_width = std::fabs(marker_width);
  output_pd.emplace_back(
      Eigen::Vector3d{marker_width / 2.0, marker_width / 2.0, 0});
  output_pd.emplace_back(
      Eigen::Vector3d{-marker_width / 2.0, marker_width / 2.0, 0});
  output_pd.emplace_back(
      Eigen::Vector3d{marker_width / 2.0, -marker_width / 2.0, 0});
  output_pd.emplace_back(
      Eigen::Vector3d{-marker_width / 2.0, -marker_width / 2.0, 0});
}
