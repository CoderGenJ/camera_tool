#include "dir_common.h"
#include "sfm.h"
#include "sfm_simulation_data_generator.h"
#include <yaml-cpp/yaml.h>
int main() {
  // 1.读取相机内参和畸变参数
  std::string camera_yaml_file = "";
  YAML::Node config = YAML::LoadFile(camera_yaml_file);
  std::string model_name = config["model_name"].as<std::string>();
  std::vector<double> intrinsic_param =
      config["intrinsic_param"].as<std::vector<double>>();
  std::vector<double> distorted_param =
      config["distorted_param"].as<std::vector<double>>();
  int resolution_w = config["resolution_w"].as<int>();
  int resolution_h = config["resolution_h"].as<int>();

  SFM::structureFromMotionConfig sfm_config;
  sfm_config.reso_x = static_cast<double>(resolution_w);
  sfm_config.reso_y = static_cast<double>(resolution_h);
  sfm_config.camera_type = model_name;
  sfm_config.distorted_param = distorted_param;
  sfm_config.intrin_param = intrinsic_param;

  SFM::structureFromMotion sfm(sfm_config);
  // 2.循环插入图片
  bool actul_data = true;
  if (actul_data) {
    std::string img_dir = "";
    std::vector<std::string> file_paths;
    dir_common::traverse_directory(img_dir, file_paths);
    for (const auto &file : file_paths) {
      if (file.length() >= 4 && file.substr(file.length() - 4) == ".png") {
        return true;
      }
      cv::Mat img = cv::imread(file);
      MarkerDetector::MarkerData marker_data;
      if (!sfm.detectMarker(img, marker_data)) {
        continue;
      }
      sfm.insertMarkerData(marker_data);
    }
  } else {
    // 1.生成虚拟数据
    data_gen::SfmDataGeneratorConfig data_gner_config;
    data_gner_config.camera_type = model_name;
    data_gner_config.debug = true;
    data_gner_config.intrinsic_param = intrinsic_param;
    data_gner_config.distort_param = distorted_param;
    data_gner_config.resolution_w = resolution_w;
    data_gner_config.resolution_h = resolution_h;
    data_gner_config.marker_width = 0.5;
    data_gner_config.pose_num = 20;
    data_gen::SfmDataGenerator sfm_data_gner(data_gner_config);
    std::vector<MarkerDetector::MarkerData> marker_datas;
    sfm_data_gner.generateData(marker_datas);
    // 2.插入graph
    for (const auto &marker_data : marker_datas) {
      sfm.insertMarkerData(marker_data);
    }
  }
  // 3.处理sfm
  sfm.optiPoseGraph();
  sfm.constructMap();
  sfm.fullBundleAdjustment();
  return 0;
}