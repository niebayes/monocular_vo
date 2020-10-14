#ifndef MONO_SLAM_UTILS_PCL_VIEWER_UTILS_H_
#define MONO_SLAM_UTILS_PCL_VIEWER_UTILS_H_

#include <array>
#include <memory>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "pcl/visualization/pcl_visualizer.h"
#include "sophus/se3.hpp"

using std::array;
using std::string;
using std::vector;

using SE3 = Sophus::SE3d;
using Vec3 = Eigen::Vector3d;
using Vec3f = Eigen::Vector3f;

namespace mono_slam {
namespace viewer_utils {

using Visualizer = pcl::visualization::PCLVisualizer;
using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
using CloudPoint = pcl::PointXYZ;
using CloudPointRGB = pcl::PointXYZRGB;
using CloudPointPtr = std::shared_ptr<CloudPoint>;
using CloudPointRGBPtr = std::shared_ptr<CloudPointRGB>;

// Color enums.
enum Color : int { BLUE = 0, GREEN = 1, RED = 2 };

class PclViewer {
 public:
  using Ptr = std::unique_ptr<PclViewer>;

  PclViewer(const Eigen::Affine3f& viewer_pose);

  ~PclViewer();

  void insertPoseEstimate(const SE3& pose, const bool is_keyframe = false);

  void insertPoseGroundTruth(const SE3& pose);

  void insertNewMapPoint(const Vec3& pos);

  void insertMapPoint(const Vec3& pos);

  void spinOnce(const int frame_id, const double scale, const int spin_time);

  void reset();

 private:
  void setupPclVisualizer(const Eigen::Affine3f& viewer_pose);

  void addPoseToTrajectory(const SE3& pose, PointCloud* point_cloud,
                           const Color& cloud_point_color,
                           const Color& line_color, const double line_width);

 private:
  int spin_cnt_;  // How many times this viewer spinned for.

  // Point cloud objects storing camera pose trajectory.
  PointCloud::Ptr pose_estimate_traj_{nullptr};
  PointCloud::Ptr pose_ground_truth_traj_{nullptr};

  // point cloud objects storing map points.
  PointCloud::Ptr all_map_points_{nullptr};
  PointCloud::Ptr new_map_points_{nullptr};

  // Pose estimate of previous frame.
  SE3 prev_pose_estimate_;
  // Pose ground truth of previous frame.
  SE3 prev_pose_ground_truth_;

  Visualizer::Ptr visualizer_{nullptr};  // PCL visualizer.
};

}  // namespace viewer_utils
}  // namespace mono_slam

#endif  // MONO_SLAM_UTILS_PCL_VIEWER_UTILS_H_