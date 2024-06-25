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
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace data_common
