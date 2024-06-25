#include "sfm.h"
namespace SFM {

void structureFromMotion::extractMarker(cv::Mat img) {
  MarkerDetector::MarkerData marker_data;
  if (!marker_detector_ptr_->detectMarker(img, marker_data)) {
    return;
  }
  for (const auto &marker_item : marker_data.id_marker_corners) {
    auto id = marker_item.first;
    auto marker_2ds = marker_item.second;
    std::vector<data_common::Point3d2dPair> pairs;
    for (size_t i = 0; i < marker_2ds.size(); ++i) {
      pairs.push_back(
          data_common::Point3d2dPair(marker_2ds.at(i), board_corners_.at(i)));
    }
    ImgNode img_node;
    img_node.index = index_;
    if (co_vis_.find(id) == co_vis_.end()) {
      co_vis_.insert(std::make_pair(id, std::vector<size_t>()));
    }
    co_vis_[id].push_back(img_node.index);
    index_++;

    Eigen::Matrix4d T_cam_board;
    if (!pnp_sovler_ptr_->solvePnP(pairs, T_cam_board)) {
      continue;
    }
    MarkerImgItem marker_img_item;
    for (size_t j = 0; j < board_corners_.size(); ++j) {
      auto &board_corner = board_corners_.at(j);
      Eigen::Vector3d pt3d_in_cam =
          T_cam_board.block<3, 3>(0, 0) *
              Eigen::Vector3d{board_corner.x, board_corner.y, board_corner.z} +
          T_cam_board.block<3, 1>(0, 3);
      marker_img_item.marker_3ds_in_cam.push_back(pt3d_in_cam);
      marker_img_item.marker_id = id;
      marker_img_item.marker_2ds.push_back(
          Eigen::Vector2d{marker_2ds.at(j).x, marker_2ds.at(j).y});
      marker_img_item.pose = data_common::Pose3d(T_cam_board);
    }
  }
}

} // namespace SFM