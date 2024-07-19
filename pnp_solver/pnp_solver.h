#include "ceres_factor.hpp"

namespace pnp_sovler {

struct PnPSolverConfig {
  size_t min_size = 4;
  int ceres_max_iter = 30;
  bool ceres_minimizer_progress_to_stdout = false;
  double pixel_diff = 2.0;
};

/// @brief 通过BA原理,基于ceres pnp求解器,需要一个良好的初始值,否则收敛不了
class PnPSolver {
public:
  PnPSolver(const PnPSolverConfig &config,
            std::shared_ptr<CameraModelNS::CameraModel> camera_model_ptr)
      : config_(config), camera_model_ptr_(camera_model_ptr) {}

  PnPSolver() {}
  /// @brief
  /// @param pt_3d_2d_pairs
  /// @param inoutput_rlt 输入的初始值
  /// @return
  bool solvePnP(const std::vector<data_common::Point3d2dPair> &pt_3d_2d_pairs,
                Eigen::Matrix4d &inoutput_rlt);
  bool checkRlt();

private:
  PnPSolverConfig config_;
  std::shared_ptr<CameraModelNS::CameraModel> camera_model_ptr_;
};

} // namespace pnp_sovler