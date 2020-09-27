#ifndef MONO_SLAM_FRONT_END_MAP_INITIALIZATION_H_
#define MONO_SLAM_FRONT_END_MAP_INITIALIZATION_H_

#include "mono_slam/common_include.h"
#include "mono_slam/frame.h"

namespace mono_slam {

class Frame;

class Initializer {
 public:
  using Ptr = sptr<Initializer>;

  enum class Stage { NO_FRAME_YET, HAS_REFERENCE_FRAME, SUCCESS };

  Initializer(const int min_num_features_init,
              const int min_num_matched_features,
              const int min_num_inlier_matches);

  void SetTracker(const sptr<Tracking>& tracker);

  void AddReferenceFrame(const Frame::Ptr& ref_frame);

  void AddCurrentFrame(const Frame::Ptr& curr_frame);

 private:
  // Compute relative pose from ref_frame to curr_frame and triangulate points
  // by the way.
  bool Initialize(const vector<int>& matches);

  // Build initial map.
  bool BuildInitMap();

  inline void Reset() {
    stage_ = Stage::NO_FRAME_YET;
    ref_frame_.reset();
    curr_frame_.reset();
    inlier_matches_.clear();
  }

  sptr<Tracking> tracker_ = nullptr;

  Stage stage_;  // Initialization stage.

  Frame::Ptr ref_frame_ = nullptr;   // Reference frame.
  Frame::Ptr curr_frame_ = nullptr;  // Current frame.

  SE3 T_curr_ref_;  // Relative pose from reference frame to current frame.
  //! points_ and inlier_matches_ are one-to-one correspondent.
  vector<Vec3> points_;  // Triangulated points in world frame.
  vector<pair<int, int>> inlier_matches_;  // Inlier matches.

  // Configuration parameters.
  const int min_num_features_init_;
  const int min_num_matched_features_;
  const int min_num_inlier_matches_;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_FRONT_END_MAP_INITIALIZATION_H_