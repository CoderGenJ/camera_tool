#include "calibration_model.h"
#include "dir_common.h"

int main(int argc, char *argv[]) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <config_yaml_path>"
              << " <img_path>" << std::endl;
    return 1;
  }

  CalibrationModelNS::PinholeCalibrationModelConfig pinhole_calibration_config;
  std::string yaml_path = argv[1];
  pinhole_calibration_config.initFromYaml(yaml_path);
  CalibrationModelNS::PinholeCalibrationModel camera_calibration_model(
      pinhole_calibration_config);

  std::string img_dir = argv[2];
  std::vector<std::string> file_paths;
  dir_common::traverse_directory(img_dir, file_paths);
  for (const auto &file : file_paths) {
    if (file.length() >= 4 && file.substr(file.length() - 4) == ".png") {
      return true;
    }
    cv::Mat img = cv::imread(file);
    camera_calibration_model.insertImgs(img);
    if (camera_calibration_model.enoughImgs()) {
      break;
    }
  }
  if (camera_calibration_model.calibration()) {
    std::cout << "calibration success" << std::endl;
  } else {
    std::cout << "calibration failed" << std::endl;
  }
  return 0;
}