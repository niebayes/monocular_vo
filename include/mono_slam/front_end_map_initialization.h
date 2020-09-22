#ifndef MONO_SLAM_FRONT_END_MAP_INITIALIZATION_H_
#define MONO_SLAM_FRONT_END_MAP_INITIALIZATION_H_

#include "mono_slam/common_include.h"
#include "mono_slam/frame.h"
#include "mono_slam/g2o_optimizer.h"
#include "mono_slam/geometry_solver.h"

namespace mono_slam {

class Frame;
class FeatureManager;

class Initializer {
 public:
  using Ptr = sptr<Initializer>;

  Initializer(const int min_num_features_init,
              const int min_num_matched_features,
              const int min_num_inlier_matches);

  bool AddReferenceFrame(const Frame::Ptr& ref_frame);

  bool AddCurrentFrame(const Frame::Ptr& curr_frame,
                       const std::vector<int>& matches_ref_curr);

  bool NormalizedFundamental8PointInit();

  inline void GetInitResult(Frame::Ptr last_frame, Frame::Ptr curr_frame,
                            std::vector<Vec3>& points) {
    SE3 identity;
    last_frame->cam_->SetPose(identity);
    curr_frame->cam_->SetPose(T_curr_ref_);
    points = points_;
  }

  inline void Reset() {
    ref_frame_ = nullptr;
    curr_frame_ = nullptr;
  }

 private:
  Frame::Ptr ref_frame_ = nullptr;
  Frame::Ptr curr_frame_ = nullptr;

  // Relative pose from reference frame to current frame.
  SE3 T_curr_ref_;
  // Triangulated points during
  std::vector<Vec3> points_;

  // Configuration parameters.
  const int min_num_features_init_;
  const int min_num_matched_features_;
  const int min_num_inlier_matches_;
};

Initializer::Initializer(const int min_num_features_init,
                         const int min_num_matched_features,
                         const int min_num_inlier_matches)
    : min_num_features_init_(min_num_features_init),
      min_num_matched_features_(min_num_matched_features),
      min_num_inlier_matches_(min_num_inlier_matches) {}

bool Initializer::AddReferenceFrame(const Frame::Ptr& ref_frame) {
  Reset();
  if (ref_frame->NumFeatures() < min_num_features_init_) return false;

  ref_frame_ = ref_frame;
}

bool Initializer::AddCurrentFrame(const Frame::Ptr& curr_frame,
                                  const std::vector<int>& matches_ref_curr) {
  if (!ref_frame_) {
    LOG(ERROR) << "No reference frame yet.";
    return false;
  }
  if (curr_frame->NumFeatures() < min_num_features_init_) return false;
  if (matches_ref_curr.size() < min_num_matched_features_) return false;

  curr_frame_ = curr_frame;

  if (!NormalizedFundamental8PointInit()) return false;
}

bool Initializer::NormalizedFundamental8PointInit() {}

}  // namespace mono_slam

#endif  // MONO_SLAM_FRONT_END_MAP_INITIALIZATION_H_