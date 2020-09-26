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
  unordered_map<int, int> inlier_matches;
  ComputeInitRelativePose(matches, inlier_matches);
  if (inlier_matches.size() < min_num_inlier_matches_)
    return;
  else
    BuildInitMap(inlier_matches);

  // If all criteria are satisfied, initialization is successful.
  stage_ = Stage::SUCCESS;
}

void ComputeInitRelativePose(const vector<int>& matches,
                             unordered_map<int, int>& inlier_matches) {
  Mat33 F;
  unordered_map<int, int> inlier_matches;
  GeometrySolver::FindFundamentalRansac(ref_frame_, curr_frame_, matches, F,
                                        inlier_matches);
  // From F to E, and get pose.
  // GeometrySolver::
}

void BuildInitMap(const unordered_map<int, int>& inlier_matches,
                  const Mat33& F) {
  //
}

}  // namespace mono_slam