#ifndef MONO_SLAM_VIEWER_H_
#define MONO_SLAM_VIEWER_H_

#include "mono_slam/common_include.h"
#include "mono_slam/map.h"
#include "mono_slam/tracking.h"
#include "utils/pcl_viewer_utils.h"

class PclViewer;

namespace mono_slam {

class Tracking;
class Map;

class Viewer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Viewer(const Eigen::Affine3f& viewer_pose, const int fps);

  void startThread();

  void stopThread();

  void informUpdate();

  void reset();

  void setTracker(sptr<Tracking> tracker);

  void setMap(sptr<Map> map);

 private:
  void drawingLoop();

  void updateMap();

  void drawTrajectory();

  void drawMapPoints();

  Eigen::Affine3f viewer_pose_;
  const int fps_;  // Camera fps which affects the drawing fps.

  Frame::Ptr last_frame_{nullptr};
  Frame::Ptr curr_frame_{nullptr};
  list<MapPoint::Ptr> points_;

  viewer_utils::PclViewer::Ptr pcl_viewer_{nullptr};  // Pcl viewer.

  // Multi-threading stuff.
  std::thread thread_;
  //! std::condition_variable_any generalizes std::condition_variable in that it
  //! supports any lock that meets the BasicLocable requirement.
  std::condition_variable_any update_cond_var_;
  std::atomic<bool> is_running_;
  std::mutex mut_;

  // FIXME Maybe weak_ptr is more suitable.
  sptr<Tracking> tracker_{nullptr};
  sptr<Map> map_{nullptr};
};

}  // namespace mono_slam

#endif  // MONO_SLAM_VIEWER_H_