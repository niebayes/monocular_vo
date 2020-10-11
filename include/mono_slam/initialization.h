#ifndef MONO_SLAM_FRONT_END_MAP_INITIALIZATION_H_
#define MONO_SLAM_FRONT_END_MAP_INITIALIZATION_H_

#include "mono_slam/common_include.h"
#include "mono_slam/frame.h"
#include "mono_slam/tracking.h"

namespace mono_slam {

class Tracking;
class Frame;

enum class Stage { NO_FRAME_YET, HAS_REFERENCE_FRAME, SUCCESS };

class Initializer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Stage stage_;                      // Initialization stage.
  Frame::Ptr ref_frame_ = nullptr;   // Reference frame.
  Frame::Ptr curr_frame_ = nullptr;  // Current frame.

 private:
  SE3 T_curr_ref_;  // Relative pose from reference frame to current frame.
  vector<pair<int, int>> inlier_matches_;  // Inlier matches.
  //! points_ and triangulate_mask_ are one-to-one correspondent.
  // Triangulated points in world frame.
  vector<Vec3, Eigen::aligned_allocator<Vec3>> points_;
  vector<bool> triangulate_mask_;  // Mark which inlier match produces good
                                   // triangulated point.

  sptr<Tracking> tracker_ = nullptr;  // Tracker.

 public:
  Initializer();

  inline const Stage& stage() const { return stage_; }

  void addReferenceFrame(Frame::Ptr ref_frame);

  void addCurrentFrame(Frame::Ptr curr_frame);

  void setTracker(sptr<Tracking> tracker);

 private:
  // Compute relative pose from ref_frame_ to curr_frame_ and triangulate points
  // by the way.
  bool initialize(const vector<int>& matches);

  // Build initial map.
  bool buildInitMap();

  void reset();
};

}  // namespace mono_slam

#endif  // MONO_SLAM_FRONT_END_MAP_INITIALIZATION_H_