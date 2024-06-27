#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
namespace CameraModelNS {

class CameraModel {
public:
  CameraModel(const std::vector<double> &intrinsic_param, int resolution_x,
              int resolution_y)
      : intrinsic_param_(intrinsic_param), resolution_x_(resolution_x),
        resolution_y_(resolution_y) {}

  virtual ~CameraModel() {}

  /// @brief
  /// 将3D点根据相机模型投影到像素平面,针对于图片已经去畸变的情况,投影过程中不考虑畸变
  /// @param point_in_camera 点在相机坐标系的3D坐标(X,Y,Z)
  /// @return 点在像素平面下的2D坐标(u,v)
  virtual Eigen::Vector2d
  project(const Eigen::Vector3d &point_in_camera) const = 0;

  /// @brief
  /// 将3D点根据相机模型投影到像素平面,针对于图片没有去畸变的情况,投影过程中会加入畸变
  /// @param point_in_camera 点在相机坐标系的3D坐标(X,Y,Z)
  /// @return 点在像素平面下的2D坐标(u,v)
  virtual Eigen::Vector2d
  projectDistorted(const Eigen::Vector3d &point_in_camera) const = 0;

  /// @brief 点投影从像素平面转换到相机坐标下归一化平面坐标
  /// @param point_in_img 点在像素平面下的坐标
  /// @return 点投影相机坐标下归一化平面坐标
  virtual Eigen::Vector2d
  liftPoint(const Eigen::Vector2d &point_in_img) const = 0;

  // 图像去畸变
  virtual cv::Mat undistortImage(const cv::Mat &distorted_image) const = 0;

  std::vector<double> getIntrinsicParam() { return intrinsic_param_; }
  int getReloX() { return resolution_x_; }
  int getReloY() { return resolution_y_; }

protected:
  std::vector<double> intrinsic_param_; //顺序:fx,fy,cx,cy
  int resolution_x_;
  int resolution_y_;
};

/// @brief 针孔摄像头
class PinholeCameraModel : public CameraModel {
public:
  PinholeCameraModel(const std::vector<double> &intrinsic_param,
                     int resolution_x, int resolution_y,
                     const std::vector<double> &distorted_param)
      : CameraModel(intrinsic_param, resolution_x, resolution_y),
        distorted_param_(distorted_param) {}

  Eigen::Vector2d
  project(const Eigen::Vector3d &point_in_camera) const override;

  Eigen::Vector2d
  projectDistorted(const Eigen::Vector3d &point_in_camera) const override;

  Eigen::Vector2d pointAddDistorted(const Eigen::Vector2d &pt) const;

  Eigen::Vector2d liftPoint(const Eigen::Vector2d &point_in_img) const override;

  cv::Mat undistortImage(const cv::Mat &distorted_image) const override;

private:
  std::vector<double> distorted_param_; // k1,k2,k3,p1,p2
};

class CameraFactory {
public:
  template <typename... Args>
  static std::shared_ptr<CameraModel>
  createCamera(const std::string &type, const std::vector<double> &intr_param,
               int reso_x, int reso_y, Args... args) {
    if (type == "Pinhole") {
      return std::make_shared<PinholeCameraModel>(intr_param, reso_x, reso_y,
                                                  args...);
    } else {
      throw std::invalid_argument("Unknown camera type");
    }
  }
};

} // namespace CameraModelNS