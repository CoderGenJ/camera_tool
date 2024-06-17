
#include "camera_model.h"
#include <gtest/gtest.h>

TEST(PinholeCameraModel, PinholeCameraModel) {
  std::vector<double> intrinsic_param{800, 800, 640, 480};
  std::vector<double> distorted_param{0.0, 0.0, 0.0, 0.0, 0.0};
  std::shared_ptr<CameraModelNS::PinholeCameraModel> cam_model(
      new CameraModelNS::PinholeCameraModel(intrinsic_param, distorted_param,
                                            1000, 2000));
  Eigen::Vector3d pt3d(1.0, 2.0, 3.0);
  Eigen::Vector2d pt3d_in_norm(pt3d.x() / pt3d.z(), pt3d.y() / pt3d.z());
  std::cout << "pt3d_in_norm:" << pt3d_in_norm.transpose() << std::endl;
  Eigen::Vector2d pt2d = cam_model->project(pt3d);
  Eigen::Vector2d pt2d_in_norm = cam_model->liftPoint(pt2d);
  std::cout << "pt2d_in_norm:" << pt2d_in_norm.transpose() << std::endl;
  EXPECT_NEAR(pt3d_in_norm.x(), pt2d_in_norm.x(), 1e-4);
  EXPECT_NEAR(pt3d_in_norm.y(), pt2d_in_norm.y(), 1e-4);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}