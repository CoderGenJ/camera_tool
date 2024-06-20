
#include "camera_model.h"

namespace CameraModelNS {

Eigen::Vector2d
PinholeCameraModel::project(const Eigen::Vector3d &point_in_camera) const {
  Eigen::Vector2d pt_image{0.0, 0.0};
  double x = 0, y = 0;
  if (std::abs(point_in_camera.z() < 0.001)) {
    x = std::numeric_limits<double>::infinity();
    y = std::numeric_limits<double>::infinity();
  } else {
    x = point_in_camera.x() / point_in_camera.z();
    y = point_in_camera.y() / point_in_camera.z();
    
  }

  pt_image.x() = intrinsic_param_[0] * x + intrinsic_param_[2];
  pt_image.y() = intrinsic_param_[1] * y + intrinsic_param_[3];
  return pt_image;
}
Eigen::Vector2d
PinholeCameraModel::pointAddDistorted(const Eigen::Vector2d &pt) const {
  double k1 = distorted_param_[0];
  double k2 = distorted_param_[1];
  double k3 = distorted_param_[2];
  double p1 = distorted_param_[3];
  double p2 = distorted_param_[4];

  double r2 = pt.x() * pt.x() + pt.y() * pt.y();
  double r4 = r2 * r2;
  double r6 = r4 * r2;
  Eigen::Vector2d distorted_pt{0.0, 0.0};
  distorted_pt.x() = pt.x() * (1.0 + k1 * r2 + k2 * r4 + k3 * r6) +
                     2.0 * p1 * pt.x() * pt.y() +
                     p2 * (r2 + 2.0 * pt.x() * pt.x());
  distorted_pt.y() = pt.y() * (1.0 + k1 * r2 + k2 * r4 + k3 * r6) +
                     p1 * (r2 + 2 * pt.y() * pt.y()) +
                     2.0 * p2 * pt.x() * pt.y();
  return distorted_pt;
}

Eigen::Vector2d PinholeCameraModel::projectDistorted(
    const Eigen::Vector3d &point_in_camera) const {
  Eigen::Vector2d pt_image{0.0, 0.0};
  Eigen::Vector2d pt_distorted{0.0, 0.0};
  double x = 0, y = 0;
  if (std::abs(point_in_camera.z() < 0.001)) {
    x = std::numeric_limits<double>::infinity();
    y = std::numeric_limits<double>::infinity();
  } else {
    x = point_in_camera.x() / point_in_camera.z();
    y = point_in_camera.y() / point_in_camera.z();
  }
  pt_distorted = pointAddDistorted(Eigen::Vector2d{x, y});

  pt_image.x() = intrinsic_param_[0] * x + intrinsic_param_[2];
  pt_image.y() = intrinsic_param_[1] * y + intrinsic_param_[3];
  return pt_image;
}

cv::Mat
PinholeCameraModel::undistortImage(const cv::Mat &distorted_image) const {
  cv::Mat cameraMatrix =
      (cv::Mat_<float>(3, 3) << intrinsic_param_[0], 0.0, intrinsic_param_[1],
       0.0, intrinsic_param_[2], 0.0, intrinsic_param_[3], 0.0, 0.0, 1.0);
  cv::Mat distCoeffs =
      (cv::Mat_<float>(1, 5) << distorted_param_[0], distorted_param_[1],
       distorted_param_[2], distorted_param_[3], distorted_param_[4]);
  cv::Mat undistortedImage;
  cv::undistort(distorted_image, undistortedImage, cameraMatrix, distCoeffs);
  return undistortedImage;
}

Eigen::Vector2d
PinholeCameraModel::liftPoint(const Eigen::Vector2d &point_in_img) const {
  Eigen::Vector2d output_pt;
  output_pt.x() =
      (point_in_img.x() - intrinsic_param_[2]) / intrinsic_param_[0];
  output_pt.y() =
      (point_in_img.y() - intrinsic_param_[3]) / intrinsic_param_[1];
  return output_pt;
}

} // namespace CameraModelNS