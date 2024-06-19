
#include <opencv2/opencv.hpp>
#include "calibration_model.h"
#include "dir_common.h"

int main() {
  //遍历读取文件夹的文件,获取图片
  CalibrationModelNS::PinholeCalibrationModelConfig calibration_config;
  calibration_config.initFromYaml(
      "/home/eric/workspace/camera_tool/calibration_model/pinhole_model.yaml");
  CalibrationModelNS::PinholeCalibrationModel calibration_model(
      calibration_config);

  std::string img_path = "";
  if (!dir_common::is_directory(img_path)) {
    return 0;
  }
  std::vector<std::string> img_files;
  dir_common::traverse_directory(img_path, img_files);

  for (const auto &img_file : img_files) {
    cv::Mat img_mat = cv::imread(img_file);
    calibration_model.insertImgs(img_mat);
  }
  calibration_model.calibration();
  return true;
}
