#pragma once
#include "camera_model.h"
#include "data_common.hpp"

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/covariance.h>
#include <ceres/local_parameterization.h>
#include <ceres/rotation.h>
namespace ceres_factor {

//求解PNP的因子
class Camera3D2DFactor {
public:
  Camera3D2DFactor(std::shared_ptr<CameraModelNS::CameraModel> camera_model,
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
    return (new ceres::AutoDiffCostFunction<Camera3D2DFactor, 2, 4, 3>(
        new Camera3D2DFactor(camera_model, corresponding_pair)));
  }

private:
  std::shared_ptr<CameraModelNS::CameraModel> camera_model_ptr_;
  data_common::Point3d2dPair correspond_pair_;
};
} // namespace ceres_factor