#include "camera_model.h"
#include "ceres_factor.hpp"
#include "transform_common.hpp"
#include <ceres/ceres.h>
#include <gtest/gtest.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

void generateBoardPoint(std::vector<Eigen::Vector3d> &output_pd,
                        double marker_width);
void calculatePnp(const std::vector<data_common::Point3d2dPair> pairs,
                  Eigen::Quaterniond &out_q, Eigen::Vector3d &out_t,
                  const std::vector<double> &intrinsic_param,
                  const std::vector<double> &distorted_param);

TEST(CERES_FACTOR, CERES_FACTOR) {
  //生成3D点
  std::vector<Eigen::Vector3d> pts_3d;
  generateBoardPoint(pts_3d, 1.0);
  //生成相机模型
  std::vector<double> intrinsic_param{800, 800, 640, 480};
  std::vector<double> distorted_param{0.0, 0.0, 0.0, 0.0, 0.0};
  std::shared_ptr<CameraModelNS::PinholeCameraModel> cam_model(
      new CameraModelNS::PinholeCameraModel(intrinsic_param, 1000, 2000,
                                            distorted_param));
  //生成生成转换
  Eigen::Matrix3d rotation =
      transform_common::eulerAngleToMatrix<double>(1.0, 1.1, 1.2);
  Eigen::Quaterniond q_true =
      transform_common::eulerAngleToQuat<double>(1.0, 1.1, 1.2);
  Eigen::Vector3d trans_true{1.2, 1.3, 1.4};
  Eigen::Quaterniond q_pnp;
  Eigen::Vector3d t_pnp;
  std::vector<data_common::Point3d2dPair> pairs;
  //生成3D-2D匹配对
  for (const auto &pt3d : pts_3d) {
    Eigen::Vector3d pt3d_in_cam = rotation * pt3d + trans_true;
    Eigen::Vector2d pt2d = cam_model->project(pt3d_in_cam);
    pairs.push_back(
        data_common::Point3d2dPair(cv::Point2d(pt2d.x(), pt2d.y()),
                                   cv::Point3d(pt3d.x(), pt3d.y(), pt3d.z())));
  }
  std::cout << "q_true (q): [" << q_true.x() << ", " << q_true.y() << ", "
            << q_true.z() << ", " << q_true.w() << "]" << std::endl;
  calculatePnp(pairs, q_pnp, t_pnp, intrinsic_param, distorted_param);
  //构建ceres
  ceres::Problem problem;
  ceres::LocalParameterization *quatParam =
      new ceres::QuaternionParameterization();
  //   Define Loss function
  ceres::LossFunction *loss_function = new ceres::CauchyLoss(0.5);

  Eigen::Quaterniond quat =
      transform_common::eulerAngleToQuat<double>(1.2, 1.2, 1.1);
  double R_camera_ref[4];
  double trans_camera_ref[3];
  R_camera_ref[0] = quat.w();
  R_camera_ref[1] = quat.x();
  R_camera_ref[2] = quat.y();
  R_camera_ref[3] = quat.z();

  trans_camera_ref[0] = 1.1;
  trans_camera_ref[1] = 1.4;
  trans_camera_ref[2] = 1.2;
  problem.AddParameterBlock(R_camera_ref, 4, quatParam);
  problem.AddParameterBlock(trans_camera_ref, 3);

  for (auto pair : pairs) {
    problem.AddResidualBlock(
        ceres_factor::ReprojectErrorFactor::Create(cam_model, pair),
        loss_function, R_camera_ref, trans_camera_ref);
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
  Eigen::Quaterniond q_rlt{R_camera_ref[0], R_camera_ref[1], R_camera_ref[2],
                           R_camera_ref[3]};
  Eigen::Vector3d trans_rlt{trans_camera_ref[0], trans_camera_ref[1],
                            trans_camera_ref[2]};
  EXPECT_NEAR(q_true.w(), q_rlt.w(), 1e-4);
  EXPECT_NEAR(q_true.x(), q_rlt.x(), 1e-4);
  EXPECT_NEAR(q_true.y(), q_rlt.y(), 1e-4);
  EXPECT_NEAR(q_true.z(), q_rlt.z(), 1e-4);
  EXPECT_NEAR(trans_true.x(), trans_rlt.x(), 1e-4);
  EXPECT_NEAR(trans_true.y(), trans_rlt.y(), 1e-4);
  EXPECT_NEAR(trans_true.z(), trans_rlt.z(), 1e-4);
  //对比优化结果
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

void calculatePnp(const std::vector<data_common::Point3d2dPair> pairs,
                  Eigen::Quaterniond &out_q, Eigen::Vector3d &out_t,
                  const std::vector<double> &intrinsic_param,
                  const std::vector<double> &distorted_param) {
  std::vector<cv::Point3d> objectPoints;
  std::vector<cv::Point2d> imagePoints;

  for (const auto &pair : pairs) {
    imagePoints.push_back(pair.pt2d);
    objectPoints.push_back(pair.pt3d);
  }

  cv::Mat cameraMatrix =
      (cv::Mat_<double>(3, 3) << intrinsic_param[0], 0, intrinsic_param[2], 0,
       intrinsic_param[1], intrinsic_param[3], 0, 0, 1);
  cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
  cv::Mat rvec, tvec;

  // 使用solvePnP求解
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
  cv::Mat R;
  cv::Rodrigues(rvec, R);

  Eigen::Matrix3d rotationMatrix;
  cv::cv2eigen(R, rotationMatrix);

  Eigen::Quaterniond q(rotationMatrix);

  // 输出结果
  std::cout << "Quaternion (q): [" << q.x() << ", " << q.y() << ", " << q.z()
            << ", " << q.w() << "]" << std::endl;
  std::cout << "Translation Vector (tvec): " << tvec << std::endl;
}