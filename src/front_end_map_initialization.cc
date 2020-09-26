#include "mono_slam/front_end_map_initialization.h"

#include "mono_slam/geometry_solver.h"

namespace mono_slam {

Initializer::Initializer(const int min_num_features_init,
                         const int min_num_matched_features,
                         const int min_num_inlier_matches)
    : state_(Stage::NO_FRAME_YET),
      min_num_features_init_(min_num_features_init),
      min_num_matched_features_(min_num_matched_features),
      min_num_inlier_matches_(min_num_inlier_matches) {}

Initializer::SetTracker(const sptr<Tracking>& tracker) { tracker_ = tracker; }

void Initializer::AddReferenceFrame(const Frame::Ptr& ref_frame) {
  Reset();
  if (ref_frame->NumObs() < min_num_features_init_) {
    ref_frame_ = ref_frame;
    stage_ = Stage::HAS_REFERENCE_FRAME;
  }
}

void Initializer::AddCurrentFrame(const Frame::Ptr& curr_frame) {
  if (stage_ != Stage::HAS_REFERENCE_FRAME) {
    LOG(ERROR) << "No reference frame yet.";
    return;
  }
  if (curr_frame->NumObs() < min_num_features_init_) return;
  curr_frame_ = curr_frame;
  // Matches between reference frame and current frame such that:
  // last_frame_[i] = curr_frame_[matches[i]].
  vector<int> matches;
  const int num_matches =
      Matcher::SearchForInitialization(last_frame_, curr_frame_, matches);
  if (num_matches < min_num_matched_features_) return;
  if (Initialize(matches) && BuildInitMap()) {
    // If all criteria are satisfied, initialization is successful.
    stage_ = Stage::SUCCESS;
  }
}

bool Initialize(const vector<int>& matches) {
  Mat33 F;
  unordered_map<int, int> inlier_matches;
  GeometrySolver::FindFundamentalRansac(ref_frame_, curr_frame_, matches, F,
                                        inlier_matches);
  if (inlier_matches.size() < min_num_inlier_matches) return false;
  // Mark which point correspondence produce good triangulation.
  vector<bool> triangulate_mask;
  GeometrySolver::FindRelativePoseRansac(ref_frame_, curr_frame_, F,
                                         inlier_matches, T_curr_ref_, points_,
                                         triangulate_mask);
  
  return true;
}

bool BuildInitMap() {
  //
  return true;
}

}  // namespace mono_slam