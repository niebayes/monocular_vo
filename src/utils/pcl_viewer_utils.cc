#include "mono_slam/utils/pcl_viewer_utils.h"

namespace viewer_utils {

// Line width specifying at which scale to render the lines.
static constexpr int kLineWidthPoseEstimate = 3, kLineWidthPoseGroundTruth = 1;

// Ids used for PCL to identify each coordinate system.
static string datum_coord_id{"datum_frame"},
    pose_estimate_coord_id{"pose_estimate"},
    pose_ground_truth_coord_id{"pose_ground_truth"};

// Scales of the axes.
static double pose_estimate_coord_axes_scale = 1.0,
              pose_ground_truth_coord_axes_scale = 1.0;

PclViewer::PclViewer() {
  pose_estimate_traj_.reset(new PointCloud());
  pose_ground_truth_traj_.reset(new PointCloud());
  all_map_points_.reset(new PointCloud());
  new_map_points_.reset(new PointCloud());
  setupPclVisualizer();
}

PclViewer::~PclViewer() {
  if (visualizer) visualizer->close();
}

void PclViewer::insertPoseEstimate(const SE3& pose, const bool is_keyframe) {
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

void PclViewer::spinOnce(const int milliseconds) {
  // Update camera info.
  static int cnt_frame = 0;

  // Update camera
  Eigen::Affine3f T_affine =
      basics::transT_CVRt_to_EigenAffine3d(cam_R_vec_, cam_t_).cast<float>();
  viewer_->removeCoordinateSystem(camera_frame_name_);
  viewer_->addCoordinateSystem(LEN_COORD_AXIS, T_affine, camera_frame_name_, 0);

  // Update truth camera
  Eigen::Affine3f T_affine_truth =
      basics::transT_CVRt_to_EigenAffine3d(truth_cam_R_vec_, truth_cam_t_)
          .cast<float>();
  viewer_->removeCoordinateSystem(truth_camera_frame_name_);
  viewer_->addCoordinateSystem(LEN_COORD_AXIS_TRUTH_TRAJ, T_affine_truth,
                               truth_camera_frame_name_, 0);

  // Update point
  for (auto cloud_info : point_clouds) {
    string name = cloud_info.first;
    CloudPtr cloud = cloud_info.second;
    viewer_->updatePointCloud(cloud, name);
  }

  // Update text
  int xpos = 20, ypos = 30;
  char str[512];
  sprintf(str, "frame id: %03d", cnt_frame);
  // sprintf(str, "frame id: %03dth\ntime: %.1fs", cnt_frame, cnt_frame/30.0);
  double r = 1, g = 1, b = 1;
  string text = str, str_id = "text display";
  int viewport = 0, fontsize = 20;
  if (cnt_frame == 0)
    viewer_->addText(text, xpos, ypos, fontsize, r, g, b, str_id, viewport);
  else
    viewer_->updateText(text, xpos, ypos, str_id);
  cnt_frame++;

  // Update point cloud info.

  visualizer_->spinOnce(milliseconds);
  // Clear cloud points cache making it ready for the next update.
  new_map_points_->points.clear();
  all_map_points_->points.clear();
}

void PclViewer::setupPclVisualizer() {
  // Set names
  viewer_name_ =
      "Trajectory{WHITE: estimated, RED: Keyframe; GREEN: truth}, Points{RED: "
      "new points} ";
  camera_frame_name_ = "camera_frame_name_";
  truth_camera_frame_name_ = "truth_camera_frame_name_";

  // Set viewer
  viewer_ = initPointCloudViewer(viewer_name_);
  viewer_->addCoordinateSystem(LEN_COORD_AXIS, "fixed world frame");
  viewer_->addCoordinateSystem(LEN_COORD_AXIS, camera_frame_name_);
  viewer_->addCoordinateSystem(LEN_COORD_AXIS_TRUTH_TRAJ,
                               truth_camera_frame_name_);

  // Add point clouds to viewer, and stored the cloud ptr into the hash. (Last
  // param is point size.)
  point_clouds[pc_cam_traj] = addPointCloud(viewer_, pc_cam_traj, 7);
  point_clouds[pc_cam_traj_ground_truth] =
      addPointCloud(viewer_, pc_cam_traj_ground_truth, 5);
  point_clouds[pc_pts_map] = addPointCloud(viewer_, pc_pts_map, 5);
  point_clouds[pc_pts_curr] = addPointCloud(viewer_, pc_pts_curr, 5);

  // Set viewer angle
  setViewerPose(*viewer_, x, y, z, rot_axis_x, rot_axis_y, rot_axis_z);

  // Set keyboard event
  viewer_->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer_)
}

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

}  // namespace viewer_utils
