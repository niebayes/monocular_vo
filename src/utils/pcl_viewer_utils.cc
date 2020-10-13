#include "mono_slam/utils/pcl_viewer_utils.h"

namespace viewer_utils {

static constexpr int kLineWidthPoseEstimate = 3, kLineWidthPoseGroundTruth = 1;

PclViewer::PclViewer() {
  pose_estimate_traj_.reset(new PointCloud());
  pose_ground_truth_traj_.reset(new PointCloud());
  all_map_points_.reset(new PointCloud());
  new_map_points_.reset(new PointCloud());
  setupPclVisualizer();
}

void PclViewer::setupPclVisualizer();

void PclViewer::addPoseToTrajectory(const SE3& pose, PointCloud* point_cloud,
                                    const Color& cloud_point_color,
                                    const Color& line_color,
                                    const int line_width) {
  // Create new pose cloud point.
  CloudPoint::Ptr pose_cloud_point;
  setCloudPointPos(pose_cloud_point.get(), pose.translation());
  // Add a line linking prev_pose_estimate_ and pose_cloud_point.
  if (prev_pose_estimate_ != nullptr) {
    linkCloudPoints(prev_pose_estimate_.get(), pose_cloud_point.get(),
                    line_color, line_width, visualizer_.get());
  }
  // Add to trajectory point cloud the new pose cloud point.
  CloudPointRGB::Ptr pose_cloud_point_rgb;
  setCloudPointPos(pose_cloud_point_rgb.get(), pose);
  setCloudPointColor(pose_cloud_point_rgb.get(), cloud_point_color);
  point_cloud->points.push_back(*pose_cloud_point_rgb);
  // Update previous pose cloud point.
  prev_pose_estimate_ = pose_cloud_point;
}

void PclViewer::insertPoseEstimate(const SE3& pose,
                                   const bool is_keyframe) {
  const Color color = is_keyframe ? BLUE : GREEN;
  addPoseToTrajectory(pose, pose_estimate_traj_.get(), color, color,
                      kLineWidthPoseEstimate);
}

void PclViewer::insertPoseGroundTruth(const SE3& pose) {
  addPoseToTrajectory(pose, pose_ground_truth_traj_.get(), RED, RED,
                      kLineWidthPoseGroundTruth);
}

void PclViewer::insertNewMapPoint(const Vec3& pos) {
  CloudPointRGB cloud_point;
  setCloudPointPos(&cloud_point, pos);
  setCloudPointColor(&cloud_point, RED);
  new_map_points_->points.push_back(cloud_point);
}

void PclViewer::insertMapPoint(const Vec3& pos) {
  CloudPointRGB cloud_point;
  setCloudPointPos(&cloud_point, pos);
  setCloudPointColor(&cloud_point, GREEN);
  all_map_points_->points.push_back(cloud_point);
}

void PclViewer::updateOnce() {
  // Clear cloud points cache making it ready for the next update.
  new_map_points_->points.clear();
  all_map_points_->points.clear();
}
}  // namespace viewer_utils
