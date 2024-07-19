#include <ceres/ceres.h>
#include <gtest/gtest.h>

#include "camera_model.h"
#include "ceres_factor.hpp"
#include "pnp_solver.h"
#include "transform_common.hpp"

void generateBoardPoint(std::vector<Eigen::Vector3d> &output_pd,
                        double marker_width);

TEST(PNP_SOVLER, PNP_SOVLER) {
  //生成3D点
  std::vector<Eigen::Vector3d> pts_3d;
  generateBoardPoint(pts_3d, 1.0);
  //生成相机模型
  std::vector<double> intrinsic_param{800, 800, 640, 480};
  std::vector<double> distorted_param{0.0, 0.0, 0.0, 0.0, 0.0};
  auto cam_model = CameraModelNS::CameraFactory::createCamera(
      "Pinhole", intrinsic_param, 1000, 2000, distorted_param);
  //   生成生成转换
  Eigen::Matrix3d rotation =
      transform_common::eulerAngleToMatrix<double>(1.0, 1.1, 1.2);
  Eigen::Quaterniond q_true =
      transform_common::eulerAngleToQuat<double>(1.0, 1.1, 1.2);
  Eigen::Vector3d trans_true{1.2, 1.3, 1.4};
  std::vector<data_common::Point3d2dPair> pairs;
  //生成3D-2D匹配对
  for (const auto &pt3d : pts_3d) {
    Eigen::Vector3d pt3d_in_cam = rotation * pt3d + trans_true;
    Eigen::Vector2d pt2d = cam_model->project(pt3d_in_cam);
    pairs.push_back(
        data_common::Point3d2dPair(cv::Point2d(pt2d.x(), pt2d.y()),
                                   cv::Point3d(pt3d.x(), pt3d.y(), pt3d.z())));
  }

  pnp_sovler::PnPSolverConfig config;
  pnp_sovler::PnPSolver solver(config, cam_model);
  Eigen::Matrix4d output_rlt;
  solver.solvePnP(pairs, output_rlt);

  //优化结果
  //   Eigen::Quaterniond q_rlt{R_camera_ref[0], R_camera_ref[1],
  //   R_camera_ref[2],
  //                            R_camera_ref[3]};
  //   Eigen::Vector3d trans_rlt{trans_camera_ref[0], trans_camera_ref[1],
  //                             trans_camera_ref[2]};
  //   EXPECT_NEAR(q_true.w(), q_rlt.w(), 1e-4);
  //   EXPECT_NEAR(q_true.x(), q_rlt.x(), 1e-4);
  //   EXPECT_NEAR(q_true.y(), q_rlt.y(), 1e-4);
  //   EXPECT_NEAR(q_true.z(), q_rlt.z(), 1e-4);
  //   EXPECT_NEAR(trans_true.x(), trans_rlt.x(), 1e-4);
  //   EXPECT_NEAR(trans_true.y(), trans_rlt.y(), 1e-4);
  //   EXPECT_NEAR(trans_true.z(), trans_rlt.z(), 1e-4);

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
