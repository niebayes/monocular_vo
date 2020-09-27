#include "mono_slam/front_end_map_initialization.h"

#include "mono_slam/frame.h"
#include "mono_slam/front_end_tracking.h"
#include "mono_slam/g2o_optimizer.h"
#include "mono_slam/geometry_solver.h"
#include "mono_slam/map.h"
#include "mono_slam/map_point.h"
#include "mono_slam/matcher.h"

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
  if (ref_frame->NumObs() < min_num_features_init_) return;
  ref_frame_ = ref_frame;
  stage_ = Stage::HAS_REFERENCE_FRAME;
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
  // Find fundamental matrix F.
  Mat33 F;
  GeometrySolver::FindFundamentalRansac(ref_frame_, curr_frame_, matches, F,
                                        inlier_matches_);
  if (inlier_matches_.size() < min_num_inlier_matches) return false;
  // Find relative pose from ref_frame_ to curr_frame_.
  if (!GeometrySolver::FindRelativePoseRansac(
          ref_frame_, curr_frame_, F, inlier_matches_, T_curr_ref_, points_))
    return false;

  // Set pose.
  ref_frame_->SetPose(SE3(Mat33::Identity(), Vec3::Zeros()));
  curr_frame_->SetPose(T_curr_ref_ * ref_frame_->Pose());
}

bool BuildInitMap() {
  // Insert initial keyframes.
  ref_frame_->SetKeyframe();
  curr_frame_->SetKeyframe();
  tracker_->map_->InsertKeyframe(ref_frame_);
  tracker_->map_->InsertKeyframe(curr_frame_);

  // Insert initial map points.
  const Frame::Features& feats_1 = ref_frame_->feats_;
  const Frame::Features& feats_2 = curr_frame_->feats_;
  const int num_inlier_matches = inlier_matches_.size();
  for (int i = 0; i < num_inlier_matches; ++i) {
    if (points_[i].empty()) continue;
    // Create new map point.
    MapPoint::Ptr point = make_shared<MapPoint>(points_[i]);
    // Add Observations.
    Feature::Ptr& feat_1 = feats_1[inlier_matches_[i].first];
    Feature::Ptr& feat_2 = feats_2[inlier_matches_[i].second];
    point->AddObservation(feat_1);
    point->AddObservation(feat_2);
    // Link features with the map point.
    feat_1->point_ = point;
    feat_2->point_ = point;
    // Update map point characteristics.
    point->UpdateBestDescriptor();
    point->UpdateMeanViewingDirection();
    // Add map point to map.
    tracker_->map_->InsertMapPoint(point);
  }

  // Update covisibility graph.
  ref_frame_->UpdateConnections();
  curr_frame_->UpdateConnections();

  // Global bundle adjustment.
  Map::Keyframes keyframes = tracker_->map_->GetAllKeyframesWithId();
  Map::MapPoints points = tracker_->map_->GetAllMapPointsWithId();
  Optimizer::GlobalBundleAdjustment(keyframes, points, tracker_->map_, 20);

  // Rescale the map such that the mean scene depth is equal to 1.0
  const double scene_median_depth = ref_frame_->ComputeSceneMedianDepth();
  if (scene_median_depth < 0) return false;
  const double scale = 1.0 / scene_median_depth;
  // Scale the baseline between ref_frame_ and curr_frame_.
  SE3 T_c_w_curr = curr_frame_->Pose();
  T_c_w_curr.translation() =
      -T_c_w_curr.rotation_matrix() *
      (ref_frame_->Pos() + scale * (curr_frame_->Pos() - ref_frame_->Pos()));
  curr_frame_->SetPose(T_c_w_curr);
  // Scale the position of map points.
  for (auto& id_point : points)
    id_point.second->SetPos(scale * id_point.second->Pos());

  return true;
}

}  // namespace mono_slam