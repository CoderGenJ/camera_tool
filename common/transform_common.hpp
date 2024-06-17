#include <Eigen/Dense>

namespace transform_common {
template <class T>
Eigen::Matrix<T, 3, 3> eulerAngleToMatrix(T roll, T pitch, T yaw) {
  return (Eigen::AngleAxis<T>(roll, Eigen::Matrix<T, 3, 1>::UnitX()) *
          Eigen::AngleAxis<T>(pitch, Eigen::Matrix<T, 3, 1>::UnitY()) *
          Eigen::AngleAxis<T>(yaw, Eigen::Matrix<T, 3, 1>::UnitZ()))
      .toRotationMatrix();
}

template <class T>
Eigen::Quaternion<T> eulerAngleToQuat(T roll, T pitch, T yaw) {
  return Eigen::Quaternion<T>(eulerAngleToMatrix<T>(roll, pitch, yaw));
}

} // namespace transform_common