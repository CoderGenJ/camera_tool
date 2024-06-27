#pragma once
#include "camera_model.h"
#include "data_common.hpp"

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/covariance.h>
#include <ceres/local_parameterization.h>
#include <ceres/rotation.h>
namespace ceres_factor {
/// @brief 该因子只适配去畸变过后的图片
class ReprojectErrorMapPoseFactor {
public:
  ReprojectErrorMapPoseFactor(
      std::shared_ptr<CameraModelNS::CameraModel> camera_model,
      const Eigen::Vector2d &feature_pt)
      : camera_model_ptr_(camera_model), feature_pt_(feature_pt) {}
  template <typename T>
  bool operator()(const T * q, const T * trans, const T * map_pt,
                  T *residual) const {
    Eigen::Matrix<T, 2, 1> feature_pt_temp = feature_pt_.cast<T>();
    Eigen::Matrix<T, 2, 1> reproj_pt;

    Eigen::Map<const Eigen::Matrix<T, 3, 1>> map_pt_temp(map_pt);
    Eigen::Map<const Eigen::Quaternion<T>> q_temp(q);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> trans_temp(trans);
    Eigen::Matrix<T, 3, 1> map_pt_in_cam = q_temp * map_pt_temp + trans_temp;

    T x = map_pt_in_cam[0] / map_pt_in_cam[2];
    T y = map_pt_in_cam[1] / map_pt_in_cam[2];
    //计算重投影
    auto intrinsic_param = camera_model_ptr_->getIntrinsicParam();
    T fx = T(intrinsic_param[0]);
    T fy = T(intrinsic_param[1]);
    T cx = T(intrinsic_param[2]);
    T cy = T(intrinsic_param[3]);
    reproj_pt.x() = fx * x + cx;
    reproj_pt.y() = fy * y + cy;
    residual[0] = reproj_pt.x() - feature_pt_temp[0];
    residual[1] = reproj_pt.y() - feature_pt_temp[1];
    return true;
  }
  static ceres::CostFunction *
  Create(std::shared_ptr<CameraModelNS::CameraModel> camera_model,
         const Eigen::Vector2d &feature_pt) {
    return (new ceres::AutoDiffCostFunction<ReprojectErrorMapPoseFactor, 2, 4,
                                            3, 3>(
        new ReprojectErrorMapPoseFactor(camera_model, feature_pt)));
  }

private:
  std::shared_ptr<CameraModelNS::CameraModel> camera_model_ptr_;
  Eigen::Vector2d feature_pt_;
};

//求解PNP的因子
class ReprojectErrorFactor {
public:
  ReprojectErrorFactor(std::shared_ptr<CameraModelNS::CameraModel> camera_model,
                       const data_common::Point3d2dPair &corresponding_pair) {
    camera_model_ptr_ = camera_model;
    correspond_pair_ = corresponding_pair;
  }
  template <typename T>
  bool operator()(const T *q, const T *trans, T *residual) const {
    //将优化变量转换成Eigen模板
    Eigen::Quaternion<T> q_camera_ref{q[0], q[1], q[2], q[3]};
    Eigen::Matrix<T, 3, 3> R_camera_ref = q_camera_ref.toRotationMatrix();
    Eigen::Matrix<T, 3, 1> trans_camera_ref{trans[0], trans[1], trans[2]};

    //将一致变量转换为Eigen模板
    Eigen::Vector2d pt_in_img{correspond_pair_.pt2d.x, correspond_pair_.pt2d.y};
    Eigen::Vector2d pt_in_norm = camera_model_ptr_->liftPoint(pt_in_img);
    Eigen::Matrix<T, 2, 1> pt_in_norm_temp = pt_in_norm.cast<T>();
    Eigen::Matrix<T, 3, 1> pt3d_temp =
        Eigen::Vector3d{correspond_pair_.pt3d.x, correspond_pair_.pt3d.y,
                        correspond_pair_.pt3d.z}
            .cast<T>();
    //构建cost
    Eigen::Matrix<T, 3, 1> pt3d_in_cam_temp =
        R_camera_ref * pt3d_temp + trans_camera_ref;

    Eigen::Matrix<T, 2, 1> pt3d_in_norm_temp{
        pt3d_in_cam_temp.x() / pt3d_in_cam_temp.z(),
        pt3d_in_cam_temp.y() / pt3d_in_cam_temp.z()};
    residual[0] = pt3d_in_norm_temp.x() - pt_in_norm_temp.x();
    residual[1] = pt3d_in_norm_temp.y() - pt_in_norm_temp.y();
    return true;
  }
  static ceres::CostFunction *
  Create(std::shared_ptr<CameraModelNS::CameraModel> camera_model,
         const data_common::Point3d2dPair &corresponding_pair) {
    return (new ceres::AutoDiffCostFunction<ReprojectErrorFactor, 2, 4, 3>(
        new ReprojectErrorFactor(camera_model, corresponding_pair)));
  }

private:
  std::shared_ptr<CameraModelNS::CameraModel> camera_model_ptr_;
  data_common::Point3d2dPair correspond_pair_;
};
using namespace data_common;

/// @brief 两个pose之间得残差
/// 维度为:6
/// 平移残差:残差相减
/// 旋转残差:旋转差的虚部
class Pose3dErrorFactor {
public:
  Pose3dErrorFactor(const Pose3d &T_ab_measured,
                    const Eigen::Matrix<double, 6, 6> &sqrt_info)
      : T_ab_measured_(T_ab_measured), sqrt_info_(sqrt_info) {}

  /// @brief 仿函数用于计算残差,其中o标定为初始坐标系
  /// @tparam T
  /// @param p_o_a frame a相当于frame o的平移
  /// @param q_o_a frame a相当于frame o的旋转
  /// @param p_o_b frame b相当于frame o的平移
  /// @param q_o_b frame b相当于frame o的旋转
  /// @return
  template <typename T>
  bool operator()(const T *const p_o_a, const T *const q_o_a,
                  const T *const p_o_b, const T *const q_o_b,
                  T *residuals_ptr) const {
    // 1.map 指针映射到eigen
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_o_a_eigen(p_o_a);
    Eigen::Map<const Eigen::Quaternion<T>> q_o_a_eigen(q_o_a);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_o_b_eigen(p_o_b);
    Eigen::Map<const Eigen::Quaternion<T>> q_o_b_eigen(q_o_b);
    // 2.计算ab相对转换
    Eigen::Quaternion<T> q_a_o = q_o_a_eigen.conjugate();
    Eigen::Quaternion<T> q_ab_estimated = q_a_o * q_o_b_eigen;
    Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_o * (p_o_b_eigen - p_o_a_eigen);
    // 3.计算残差
    Eigen::Quaternion<T> delta_q =
        T_ab_measured_.q.template cast<T>() * q_ab_estimated.conjugate();
    Eigen::Matrix<T, 3, 1> delta_t =
        p_ab_estimated - T_ab_measured_.p.template cast<T>();

    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
    residuals.template block<3, 1>(0, 0) = delta_t;
    residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();
    residuals.applyOnTheLeft(sqrt_info_.template cast<T>());
    return true;
  }

  static ceres::CostFunction *
  Create(const Pose3d &t_ab_measured,
         const Eigen::Matrix<double, 6, 6> &sqrt_information) {
    return new ceres::AutoDiffCostFunction<Pose3dErrorFactor, 6, 3, 4, 3, 4>(
        new Pose3dErrorFactor(t_ab_measured, sqrt_information));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  Pose3d T_ab_measured_;
  Eigen::Matrix<double, 6, 6> sqrt_info_;
};

} // namespace ceres_factor