#include <opencv2/opencv.hpp>

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

  bool insertImgs(const cv::Mat &img);
  bool enoughImgs() const { return imgs_files_.size() >= img_mini_num_; }
  virtual bool calibration() const = 0;
  virtual bool displayCalibRlt(const std::string file_path) const = 0;
  std::vector<double> getIntrinsic() { return intrinsics_; }
  std::vector<double> getDistortedParam() { return distorted_param_; }

protected:
  std::vector<std::string> imgs_files_;
  std::vector<double> intrinsics_;
  std::vector<double> distorted_param_;

  size_t img_mini_num_;
  size_t intrinsics_num_;
  size_t distorted_param_num_;
};

class PinholeCalibrationModel : public BaseCalibrationModel {
public:
  PinholeCalibrationModel(size_t img_mini_num, size_t intrinsics_num,
                          size_t distorted_param_num)
      : BaseCalibrationModel(img_mini_num, intrinsics_num,
                             distorted_param_num) {}
  bool calibration() const override;
};

} // namespace CalibrationModelNS
