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

  // Initialize using normalized eight-point algorithm.
  // Return number of inliers.
  int NormalizedFundamental8PointInit();

  inline void Reset() {
    stage_ = Stage::NO_FRAME_YET;
    ref_frame_.reset();
    curr_frame_.reset();
  }

 private:
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

Initializer::Initializer(const int min_num_features_init,
                         const int min_num_matched_features,
                         const int min_num_inlier_matches)
    : state_(Stage::NO_FRAME_YET),
      min_num_features_init_(min_num_features_init),
      min_num_matched_features_(min_num_matched_features),
      min_num_inlier_matches_(min_num_inlier_matches) {}

void Initializer::AddReferenceFrame(const Frame::Ptr& ref_frame) {
  Reset();
  if (ref_frame->NumObservations() < min_num_features_init_) {
    ref_frame_ = ref_frame;
    stage_ = Stage::HAS_REFERENCE_FRAME;
  }
}

void Initializer::AddCurrentFrame(const Frame::Ptr& curr_frame, ) {
  if (stage_ != Stage::HAS_REFERENCE_FRAME) {
    LOG(ERROR) << "No reference frame yet.";
    return;
  }
  if (curr_frame->NumObservations() < min_num_features_init_) return;
  curr_frame_ = curr_frame;
  // Matches between reference frame and current frame such that:
  // last_frame_[i] = curr_frame_[matches[i]].
  vector<int> matches;
  const int num_matches =
      Matcher::SearchForInitialization(last_frame_, curr_frame_, matches);
  if (num_matches < min_num_matched_features_) return;
  const int num_inlier_matches = NormalizedFundamental8PointInit();
  if (num_inlier_matches < min_num_inlier_matches_) return;

  // If all criteria are satisfied, initialization is successful.
  stage_ = Stage::SUCCESS;
}

int Initializer::NormalizedFundamental8PointInit() {}

}  // namespace mono_slam

#endif  // MONO_SLAM_FRONT_END_MAP_INITIALIZATION_H_