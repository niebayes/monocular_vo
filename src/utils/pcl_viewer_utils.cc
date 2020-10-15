#include "mono_slam/utils/pcl_viewer_utils.h"

#include "mono_slam/utils/math_utils.h"

using namespace pcl::visualization;

namespace mono_slam {
namespace viewer_utils {

static const vector<array<int, 3>> colors{
    {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 255}};

// Coordinate ids.
static const string pose_estimate_coord{"pose_estimate_coord"},
    pose_ground_truth_coord{"pose_ground_truth_coord"};

// Point cloud ids.
static const string pose_estimate{"pose_estimate"},
    pose_ground_truth{"pose_ground_truth"}, all_map_points{"all_map_points"},
    new_map_points{"new_map_points"};

namespace pcl_utils {

template <typename CloudPointType>
static inline void setCloudPointPos(CloudPointType* cloud_point,
                                    const Vec3& pos) {
  cloud_point->x = pos(0);
  cloud_point->y = pos(1);
  cloud_point->z = pos(2);
}

static inline void setCloudPointColor(CloudPointRGB* cloud_point,
                                      const Color& color) {
  const array<int, 3>& color_value = colors.at(color);
  cloud_point->r = color_value[0];
  cloud_point->g = color_value[1];
  cloud_point->b = color_value[2];
}

static inline void linkCloudPoints(CloudPoint* cloud_point_1,
                                   CloudPoint* cloud_point_2,
                                   const Color& line_color,
                                   const double line_width,
                                   Visualizer* visualizer) {
  const array<int, 3>& color_value = colors.at(line_color);
  static int line_id = 0;
  visualizer->addLine<CloudPoint>(*cloud_point_1, *cloud_point_2,
                                  color_value[0], color_value[1],
                                  color_value[2], std::to_string(line_id));
  visualizer->setShapeRenderingProperties(PCL_VISUALIZER_LINE_WIDTH, line_width,
                                          std::to_string(line_id));
  ++line_id;
}

static inline void addPointCloud(const Visualizer::Ptr& visualizer,
                                 const PointCloud::Ptr& point_cloud,
                                 const string& cloud_id,
                                 const double cloud_point_size) {
  // Color handler for this point cloud.
  PointCloudColorHandlerRGBField<CloudPointRGB> color_handler(point_cloud);
  visualizer->addPointCloud<CloudPointRGB>(point_cloud, color_handler,
                                           cloud_id);
  visualizer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE,
                                               cloud_point_size, cloud_id);
}

//! This function is copied from pcl tutorial.
//@see
// https://pcl.readthedocs.io/projects/tutorials/en/latest/range_image_visualization.html#range-image-visualization
static inline void setViewerPose(const Visualizer::Ptr& visualizer,
                                 const Eigen::Affine3f& viewer_pose) {
  const Vec3f pos_vector = viewer_pose * Vec3f{0., 0., 0.};
  const Vec3f look_at_vector =
      viewer_pose.rotation() * Vec3f{0., 0., 1.} + pos_vector;
  const Vec3f up_vector = viewer_pose.rotation() * Vec3f{0., -1., 0.};
  // Set camera position and orientation at which the viewer is observing the
  // point cloud world.
  //! Refer to vtk's camera model for more detail.
  visualizer->setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
                                look_at_vector[0], look_at_vector[1],
                                look_at_vector[2], up_vector[0], up_vector[1],
                                up_vector[2]);
}

}  // namespace pcl_utils

using namespace pcl_utils;

PclViewer::PclViewer(const Eigen::Affine3f& viewer_pose) : spin_cnt_(0) {
  pose_estimate_traj_.reset(new PointCloud());
  pose_ground_truth_traj_.reset(new PointCloud());
  all_map_points_.reset(new PointCloud());
  new_map_points_.reset(new PointCloud());
  setupPclVisualizer(viewer_pose);
}

PclViewer::~PclViewer() {
  if (visualizer_) visualizer_->close();
}

void PclViewer::insertPoseEstimate(const SE3& pose, const bool is_keyframe) {
  const Color color = is_keyframe ? Color::BLUE : Color::WHITE;
  addPoseToTrajectory(pose, pose_estimate_traj_.get(), color, color, 3.0);
  // Update previous pose estimate.
  prev_pose_estimate_ = pose;
}

void PclViewer::insertPoseGroundTruth(const SE3& pose) {
  addPoseToTrajectory(pose, pose_ground_truth_traj_.get(), Color::RED,
                      Color::RED, 1.0);
  // Update previous pose ground truth.
  prev_pose_ground_truth_ = pose;
}

void PclViewer::insertNewMapPoint(const Vec3& pos) {
  CloudPointRGB cloud_point;
  setCloudPointPos(&cloud_point, pos);
  setCloudPointColor(&cloud_point, Color::RED);
  new_map_points_->points.push_back(cloud_point);
}

void PclViewer::insertMapPoint(const Vec3& pos) {
  CloudPointRGB cloud_point;
  setCloudPointPos(&cloud_point, pos);
  setCloudPointColor(&cloud_point, Color::GREEN);
  all_map_points_->points.push_back(cloud_point);
}

void PclViewer::spinOnce(const int frame_id, const double scale,
                         const int spin_time) {
  // Refresh the coordinate systems.
  //! Currently, previous poses are actually the current poses.
  visualizer_->removeCoordinateSystem(pose_estimate_coord);
  visualizer_->addCoordinateSystem(
      1.0, math_utils::SE3_to_affine(prev_pose_estimate_), pose_estimate_coord);
  visualizer_->removeCoordinateSystem(pose_ground_truth_coord);
  visualizer_->addCoordinateSystem(
      scale, math_utils::SE3_to_affine(prev_pose_ground_truth_),
      pose_ground_truth_coord);
  //! Since mono vo is inherently by no means able to get the scale of the
  //! scene, it's required to scale the ground truth trajectory by a certain
  //! factor to align the trajectories and then the comparison can be made on
  //! the performance of our mono vo system.

  // Update point clouds.
  visualizer_->updatePointCloud(pose_estimate_traj_, pose_estimate);
  visualizer_->updatePointCloud(pose_ground_truth_traj_, pose_ground_truth);
  visualizer_->updatePointCloud(all_map_points_, all_map_points);
  visualizer_->updatePointCloud(new_map_points_, new_map_points);

  // Update text
  string info{"frame id: "};
  info.append(std::to_string(frame_id));
  if (spin_cnt_ == 0)
    visualizer_->addText(info, 20, 20, 20, 1.0, 1.0, 1.0, "text id");
  else
    visualizer_->updateText(info, 20, 20, "text id");

  // Update pcl visualizer.
  visualizer_->spinOnce(spin_time);

  // Clear cloud points cache making'em ready for the next update.
  new_map_points_->points.clear();
  all_map_points_->points.clear();
}

void PclViewer::reset() {
  pose_estimate_traj_->clear();
  pose_ground_truth_traj_->clear();
  all_map_points_->clear();
  new_map_points_->clear();
}

void PclViewer::setupPclVisualizer(const Eigen::Affine3f& viewer_pose) {
  // Init visualizer.
  visualizer_.reset(new Visualizer());
  visualizer_->setWindowName(
      "Trajectory: {White: pose estimate, Red: pose ground truth, Blue: "
      "keyframe}, Points: {Green: map point, Red: new map point}");
  visualizer_->setBackgroundColor(0, 0, 0);  // Black.
  visualizer_->initCameraParameters();
  // Set viewer pose at which the point cloud is observed by.
  setViewerPose(visualizer_, viewer_pose);

  // Add to the visualizer coordinate systems based on which the point clouds
  // are specified.
  visualizer_->addCoordinateSystem(1.0, "datum_frame_coord");
  visualizer_->addCoordinateSystem(1.0, pose_estimate_coord);
  visualizer_->addCoordinateSystem(1.0, pose_ground_truth_coord);

  // Add to the visualizer the point clouds.
  addPointCloud(visualizer_, pose_estimate_traj_, pose_estimate, 7.0);
  addPointCloud(visualizer_, pose_ground_truth_traj_, pose_ground_truth, 5.0);
  addPointCloud(visualizer_, all_map_points_, all_map_points, 5.0);
  addPointCloud(visualizer_, new_map_points_, new_map_points, 5.0);

  // Register keyboard event
  // visualizer_->registerKeyboardCallback(keyboardEventOccurred,
  // (void*)&visualizer_)
}

void PclViewer::addPoseToTrajectory(const SE3& pose, PointCloud* point_cloud,
                                    const Color& cloud_point_color,
                                    const Color& line_color,
                                    const double line_width) {
  // Create new pose cloud point.
  using std::make_unique;
  CloudPointPtr curr_pose_cloud_point = make_unique<CloudPoint>();
  setCloudPointPos(curr_pose_cloud_point.get(), pose.translation());
  // Add a line linking previous pose cloud point and current pose cloud point.
  CloudPointPtr prev_pose_cloud_point = make_unique<CloudPoint>();
  if (spin_cnt_ > 0) {  // If not the first time the viewer is invoked.
    setCloudPointPos(prev_pose_cloud_point.get(),
                     prev_pose_estimate_.translation());
    linkCloudPoints(prev_pose_cloud_point.get(), curr_pose_cloud_point.get(),
                    line_color, line_width, visualizer_.get());
  }
  // Add to trajectory point cloud the new pose cloud point.
  CloudPointRGBPtr pose_cloud_point_rgb = make_unique<CloudPointRGB>();
  setCloudPointPos(pose_cloud_point_rgb.get(), pose.translation());
  setCloudPointColor(pose_cloud_point_rgb.get(), cloud_point_color);
  point_cloud->points.push_back(*pose_cloud_point_rgb);
}

}  // namespace viewer_utils
}  // namespace mono_slam
