#ifndef MONO_SLAM_UTILS_PCL_VIEWER_UTILS_H_
#define MONO_SLAM_UTILS_PCL_VIEWER_UTILS_H_

#include <array>
#include <vector>

#include "pcl/visualization/pcl_visualizer.h"
#include "sophus/se3.hpp"

using std::array;
using std::string;
using std::vector;

using SE3 = Sophus::SE3d;
using Vec3 = Sophus::Vector3d;

namespace viewer_utils {

using Visualizer = pcl::visualization::PCLVisualizer;
using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
using CloudPoint = pcl::PointXYZ;
using CloudPointRGB = pcl::PointXYZRGB;

enum class Color : int { BLUE, GREEN, RED };

static constexpr array<array<double, 3>, 3> RGB{
    {255., 0., 0.}, {0., 255., 0.}, {0., 0., 255.}};

class PclViewer {
 public:
  PclViewer();

  void setupPclVisualizer();

  void addPoseToTrajectory(const SE3& pose, PointCloud* point_cloud,
                           const Color& cloud_point_color,
                           const Color& line_color, const int line_width);

  void insertPoseEstimate(const SE3& pose, const bool is_keyframe = false);

  void insertPoseGroundTruth(const SE3& pose);

  void insertNewMapPoint(const Vec3& pos);

  void insertMapPoint(const Vec3& pos);

  void updateOnce();

 private:
  // Point cloud objects storing camera pose trajectory.
  PointCloud::Ptr pose_estimate_traj_{nullptr};
  PointCloud::Ptr pose_ground_truth_traj_{nullptr};

  // point cloud objects storing map points.
  PointCloud::Ptr all_map_points_{nullptr};
  PointCloud::Ptr new_map_points_{nullptr};

  // Pose estimate of previous frame.
  CloudPoint::Ptr prev_pose_estimate_{nullptr};
  // Pose ground truth of previous frame.
  CloudPoint::Ptr prev_pose_ground_truth_{nullptr};

  Visualizer::Ptr visualizer_{nullptr};  // PCL visualizer.
};

namespace pcl_utils {

template <typename CloudPointType>
static inline void setCloudPointPos(CloudPointType* cloud_point,
                                    const Vec3& pos) {
  cloud_point->x = pos(0);
  cloud_point->y = pos(0);
  cloud_point->z = pos(0);
}

static inline void setCloudPointColor(CloudPointRGB* cloud_point,
                                      const Color& color) {
  const array<double, 3>& color = RGB.at(color);
  cloud_point->r = color[0];
  cloud_point->g = color[1];
  cloud_point->b = color[2];
}

static inline void linkCloudPoints(CloudPoint* cloud_point_1,
                                   CloudPoint* cloud_point_2,
                                   const Color& line_color,
                                   const int line_width,
                                   Visualizer* visualizer) {
  const array<double, 3>& color = RGB.at(color);
  visualizer->addLine<CloudPoint>(*cloud_point_1, *cloud_point_2, color[0],
                                  color[1], color[2]);
  visualizer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width);
}

}  // namespace pcl_utils

}  // namespace viewer_utils

#endif  // MONO_SLAM_UTILS_PCL_VIEWER_UTILS_H_