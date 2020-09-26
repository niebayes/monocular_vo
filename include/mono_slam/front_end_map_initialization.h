#ifndef MONO_SLAM_FRONT_END_MAP_INITIALIZATION_H_
#define MONO_SLAM_FRONT_END_MAP_INITIALIZATION_H_

#include "mono_slam/common_include.h"
#include "mono_slam/frame.h"
#include "mono_slam/g2o_optimizer.h"
#include "mono_slam/geometry_solver.h"
#include "mono_slam/matcher.h"

namespace mono_slam {

class Frame;

class Initializer {
 public:
  using Ptr = sptr<Initializer>;

  enum class Stage { NO_FRAME_YET, HAS_REFERENCE_FRAME, SUCCESS };

  Initializer(const int min_num_features_init,
              const int min_num_matched_features,
              const int min_num_inlier_matches);

  void AddReferenceFrame(const Frame::Ptr& ref_frame);

  void AddCurrentFrame(const Frame::Ptr& curr_frame);

 private:
  // Compute relative pose from reference frame to current frame and fix
  // reference frame as world frame.
  // Return inlier matches.
  vector<int> ComputeInitRelativePose(const vector<int>& matches);

  // Build initial map (i.e. triangulate 3D points).
  void BuildInitMap(const vector<int>& inlier_matches);

  inline void Reset() {
    stage_ = Stage::NO_FRAME_YET;
    ref_frame_.reset();
    curr_frame_.reset();
  }

  Stage stage_;  // Initialization stage.

  Frame::Ptr ref_frame_ = nullptr;   // Reference frame.
  Frame::Ptr curr_frame_ = nullptr;  // Current frame.

  vector<int> inlier_matches_;  // Inlier matches survived after initialization.
  SE3 T_curr_ref_;       // Relative pose from reference frame to current frame.
  vector<Vec3> points_;  // Triangulated points in world frame.

  // Configuration parameters.
  const int min_num_features_init_;
  const int min_num_matched_features_;
  const int min_num_inlier_matches_;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_FRONT_END_MAP_INITIALIZATION_H_