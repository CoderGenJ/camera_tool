#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
namespace data_common {
struct Point3d2dPair {
  Point3d2dPair(const cv::Point2d &_pt2d, const cv::Point3d &_pt3d)
      : pt2d(_pt2d), pt3d(_pt3d) {}

  Point3d2dPair() : pt2d(0.0, 0.0), pt3d(0.0, 0.0, 0.0) {}

  cv::Point2d pt2d;
  cv::Point3d pt3d;
};

struct Pose3d {
  Pose3d() {}
  Pose3d(const Eigen::Matrix4d &T) {
    p = T.block<3, 1>(0, 3);
    q = Eigen::Quaterniond(T.block<3, 3>(0, 0));
    q.normalize();
  }
  /// @brief 求pose的逆
  /// @param
  /// @return
  Pose3d inverse() const {
    Pose3d result;
    result.q = q.inverse();
    result.p = -1.0 * (q.inverse() * p);
    return result;
  }

  Pose3d operator*(const Pose3d &other) const {
    Pose3d result;
    result.q = q * other.q;
    result.p = q * other.p + p;
    return result;
  }

  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace data_common
