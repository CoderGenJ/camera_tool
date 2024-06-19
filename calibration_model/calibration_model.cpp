#include "calibration_model.h"
#include <fstream>
#include <opencv2/opencv.hpp>
namespace CalibrationModelNS {
// ref:主要就是调接口
// https://blog.csdn.net/Goodness2020/article/details/126254894
bool PinholeCalibrationModel::calibration() const {
  if (!enoughImgs()) {
    std::cout << "图片数量不足" << std::endl;
    return false;
  }
  // 1.提取角点
  int image_nums = 0; // 有效图片数量统计
  cv::Size image_size(config_.height, config_.width);
  cv::Size corner_size(config_.board_corner_row, config_.board_corner_col);
  cv::Mat image_raw, image_gray;
  std::vector<cv::Point2f> points_per_image; // 缓存每幅图检测到的角点
  std::vector<std::vector<cv::Point2f>> corner_pts_all_images;
  for (const auto &img_raw : imgs_mats_) {
    cv::cvtColor(image_raw, image_gray, cv::COLOR_BGR2GRAY);
    bool success =
        cv::findChessboardCorners(image_gray, corner_size, points_per_image);
    if (!success) {
      continue;
    } else {
      cv::find4QuadCornerSubpix(image_gray, points_per_image, cv::Size(5, 5));
      corner_pts_all_images.push_back(points_per_image);
      image_nums++;
    }
  }
  // 2.计算3D点
  cv::Size block_size(config_.board_side_length, config_.board_side_length);
  std::vector<cv::Point3f> points3D_per_image;
  for (int i = 0; i < corner_size.height; i++) // 第i行---y
  {
    for (int j = 0; j < corner_size.width; j++) // 第j列---x
    {
      cv::Point3f point3D(block_size.width * j, block_size.height * i, 0);
      points3D_per_image.push_back(point3D);
    }
  }
  std::vector<std::vector<cv::Point3f>> all_borads_pt3d(image_nums,
                                                        points3D_per_image);
  int point_counts = corner_size.area();
  // 3.标定
  std::cout << "开始标定相机" << std::endl;              // calibrateCamera
  cv::Mat cameraMat(3, 3, CV_32FC1, cv::Scalar::all(0)); // 内参矩阵3*3
  cv::Mat distCoeffs(1, 5, CV_32FC1,
                     cv::Scalar::all(0)); // 畸变矩阵1*5
  std::vector<cv::Mat> rotationMat;       // 旋转矩阵
  std::vector<cv::Mat> translationMat;    // 平移矩阵
  cv::calibrateCamera(all_borads_pt3d, corner_pts_all_images, image_size,
                      cameraMat, distCoeffs, rotationMat, translationMat, 0);

  // 4.计算标定重投影误差
  double total_err = 0.0;                    // 所有图像平均误差总和
  double err = 0.0;                          // 每幅图像的平均误差
  std::vector<cv::Point2f> points_reproject; // 重投影点
  for (int i = 0; i < image_nums; i++) {
    points_per_image = corner_pts_all_images[i]; // 第i张图像提取角点
    points3D_per_image = all_borads_pt3d[i]; // 第i张图像中角点的3D坐标
    cv::projectPoints(points3D_per_image, rotationMat[i], translationMat[i],
                      cameraMat, distCoeffs, points_reproject); // 重投影
    cv::Mat detect_points_Mat(
        1, points_per_image.size(),
        CV_32FC2); // 变为1*S的矩阵,2通道保存提取角点的像素坐标
    cv::Mat points_reproj_Mat(
        1, points_reproject.size(),
        CV_32FC2); // 变为1*S的矩阵,2通道保存投影角点的像素坐标
    for (int j = 0; j < points_per_image.size(); j++) {
      detect_points_Mat.at<cv::Vec2f>(0, j) =
          cv::Vec2f(points_per_image[j].x, points_per_image[j].y);
      points_reproj_Mat.at<cv::Vec2f>(0, j) =
          cv::Vec2f(points_reproject[j].x, points_reproject[j].y);
    }
    err = norm(points_reproj_Mat, detect_points_Mat,
               cv::NormTypes::NORM_L2); // 计算两者之间的误差
    total_err += err /= point_counts;
  }
  // 5.输出标定文件
  std::ofstream fout(config_.calibration_output_file);
  double reproj_average_error = total_err / image_nums;
  std::cout << "5、将标定结果写入文件……" << std::endl;
  fout << "相机内参数矩阵:" << std::endl << cameraMat << std::endl << std::endl;
  fout << "相机的畸变系数:" << std::endl
       << distCoeffs << std::endl
       << std::endl;
  fout << "总体平均误差为： " << reproj_average_error << "像素" << std::endl
       << std::endl;
  fout.close();
  if (total_err / image_nums > config_.reproj_error_th) {
    std::cout << "重投影误差太大" << reproj_average_error << std::endl;
    return false;
  }
  return true;
}

} // namespace CalibrationModelNS
