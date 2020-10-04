#ifndef MONO_SLAM_FRONT_END_MAP_INITIALIZATION_H_
#define MONO_SLAM_FRONT_END_MAP_INITIALIZATION_H_

#include "mono_slam/common_include.h"
#include "mono_slam/frame.h"
#include "mono_slam/tracking.h"

namespace mono_slam {

class Tracking;
class Frame;

class Initializer {
 public:
  enum class Stage { NO_FRAME_YET, HAS_REFERENCE_FRAME, SUCCESS };

 private:
  sptr<Tracking> tracker_ = nullptr;

  Stage stage_;  // Initialization stage.

  Frame::Ptr ref_frame_ = nullptr;   // Reference frame.
  Frame::Ptr curr_frame_ = nullptr;  // Current frame.

  SE3 T_curr_ref_;  // Relative pose from reference frame to current frame.
  //! points_ and triangulate_mask_ are one-to-one correspondent.
  vector<Vec3> points_;            // Triangulated points in world frame.
  vector<pair<int, int>> inlier_matches_;  // Inlier matches.
  vector<bool> triangulate_mask_;  // Mark which inlier match produces good
                                   // triangulated point.

 public:
  Initializer();

  inline const Stage& stage() const { return stage_; }

  void setTracker(sptr<Tracking> tracker);

  void addReferenceFrame(Frame::Ptr ref_frame);

  void addCurrentFrame(Frame::Ptr curr_frame);

 private:
  // Compute relative pose from ref_frame_ to curr_frame_ and triangulate points
  // by the way.
  bool initialize(const vector<int>& matches);

  // Build initial map.
  bool buildInitMap();

  inline void reset() {
    stage_ = Stage::NO_FRAME_YET;
    ref_frame_.reset();
    curr_frame_.reset();
    inlier_matches_.clear();
  }
};

}  // namespace mono_slam

#endif  // MONO_SLAM_FRONT_END_MAP_INITIALIZATION_H_