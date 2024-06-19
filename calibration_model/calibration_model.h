#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

namespace CalibrationModelNS {
class BaseCalibrationModel {
public:
  BaseCalibrationModel(size_t img_mini_num, size_t intrinsics_num,
                       size_t distorted_param_num)
      : img_mini_num_(img_mini_num), intrinsics_num_(intrinsics_num),
        distorted_param_num_(distorted_param_num) {
    intrinsics_.resize(intrinsics_num_, 0.0);
    distorted_param_.resize(distorted_param_num_, 0.0);
  }
  virtual ~BaseCalibrationModel() {}

  bool insertImgs(const cv::Mat &img) {
    if (img.empty()) {
      return false;
    }
    imgs_mats_.push_back(img);
    return true;
  }
  bool enoughImgs() const { return imgs_mats_.size() >= img_mini_num_; }
  virtual bool calibration() const = 0;
  // virtual bool displayCalibRlt(const std::string file_path) const = 0;
  std::vector<double> getIntrinsic() { return intrinsics_; }
  std::vector<double> getDistortedParam() { return distorted_param_; }

protected:
  std::vector<cv::Mat> imgs_mats_;
  std::vector<double> intrinsics_;
  std::vector<double> distorted_param_;

  size_t img_mini_num_;
  size_t intrinsics_num_;
  size_t distorted_param_num_;
};

struct PinholeCalibrationModelConfig {
  PinholeCalibrationModelConfig() {}
  bool initFromYaml(const std::string &yaml_file) {
    try {
      YAML::Node config = YAML::LoadFile(yaml_file);
      if (config["board_corner_row"]) {
        board_corner_row = config["board_corner_row"].as<int>();
      }
      if (config["board_corner_col"]) {
        board_corner_col = config["board_corner_col"].as<int>();
      }
      if (config["board_side_length"]) {
        board_side_length = config["board_side_length"].as<int>();
      }
      if (config["width"]) {
        width = config["width"].as<int>();
      }
      if (config["height"]) {
        height = config["height"].as<int>();
      }

      if (config["img_mini_num"]) {
        img_mini_num = config["img_mini_num"].as<size_t>();
      }
      if (config["intrinsics_num"]) {
        intrinsics_num = config["intrinsics_num"].as<size_t>();
      }
      if (config["distorted_param_num"]) {
        distorted_param_num = config["distorted_param_num"].as<size_t>();
      }

      if (config["reproj_error_th"]) {
        reproj_error_th = config["reproj_error_th"].as<double>();
      }
      if (config["calibration_output_file"]) {
        calibration_output_file =
            config["calibration_output_file"].as<std::string>();
      }
    } catch (const YAML::Exception &e) {
      std::cerr << "Error reading YAML file: " << e.what() << std::endl;
      return false;
    }
    return true;
  }
  int board_corner_row;  // 棋盘格行数
  int board_corner_col;  // 棋盘格列数
  int board_side_length; //棋盘格正方形边长
  int width;             //图片分辨率 宽
  int height;            //图片分辨率 高

  size_t img_mini_num;        //图片个数
  size_t intrinsics_num;      //内参个数
  size_t distorted_param_num; //畸变个数

  double reproj_error_th; //重投影误差阈值
  std::string calibration_output_file = "";
};

class PinholeCalibrationModel : public BaseCalibrationModel {
public:
  PinholeCalibrationModel(const PinholeCalibrationModelConfig &config)
      : BaseCalibrationModel(config.img_mini_num, config.intrinsics_num,
                             config.distorted_param_num),
        config_(config) {}
  bool calibration() const override;

private:
  PinholeCalibrationModelConfig config_;
};

} // namespace CalibrationModelNS
