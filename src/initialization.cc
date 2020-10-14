#include "mono_slam/initialization.h"

#include "mono_slam/g2o_optimizer.h"
#include "mono_slam/geometry_solver.h"
#include "mono_slam/map.h"
#include "mono_slam/map_point.h"
#include "mono_slam/matcher.h"

namespace mono_slam {

Initializer::Initializer() : stage_(Stage::NO_FRAME_YET) {}

void Initializer::addReferenceFrame(Frame::Ptr ref_frame) {
  LOG(INFO) << "Start initialization.";
  reset();
  if (ref_frame->nObs() < Config::init_min_n_feats()) return;
  ref_frame_ = ref_frame;
  LOG(INFO) << "Reference frame selected.";
  stage_ = Stage::HAS_REFERENCE_FRAME;
}

void Initializer::addCurrentFrame(Frame::Ptr curr_frame) {
  if (stage_ != Stage::HAS_REFERENCE_FRAME) {
    LOG(ERROR) << "No reference frame yet.";
    return;
  }
  if (curr_frame->nObs() < Config::init_min_n_feats()) return;
  // Ensure the baseline between the two frames used for initialization is
  // sufficient large or the initialization quality is bad.
  // if (curr_frame->id_ < ref_frame_->id_ + Config::new_kf_interval()) return;
  curr_frame_ = curr_frame;
  LOG(INFO) << "Current frame selected.";
  // Matches between reference frame and current frame such that:
  // last_frame_[i] = curr_frame_[matches[i]].
  vector<int> matches;
  const int n_matches =
      Matcher::searchForInitialization(ref_frame_, curr_frame_, matches);
  LOG(INFO) << "matches(ref_frame_, curr_frame_) = " << n_matches;
  if (n_matches < Config::init_min_n_matches()) {
    // Two consecutive good frames are necessary for good initialization.
    LOG(INFO) << "Halt initialization.";
    reset();
    return;
  }
  if (initialize(matches) && buildInitMap()) {
    // If all criteria are satisfied, initialization is successful.
    LOG(INFO) << "Initialization succeeded.";
    stage_ = Stage::SUCCESS;
  } else {
    LOG(INFO) << "Initialization failed.";
    reset();
  }
}

bool Initializer::initialize(const vector<int>& matches) {
  // Find fundamental matrix F.
  Mat33 F;
  GeometrySolver::findFundamentalRansac(ref_frame_, curr_frame_, matches, F,
                                        inlier_matches_);
  LOG(INFO) << "inlier_matches(ref_frame_, curr_frame_) = "
            << inlier_matches_.size();
  if (inlier_matches_.size() < Config::init_min_n_inlier_matches()) {
    LOG(INFO) << "Insufficient inlier matches, retry initialization.";
    return false;
  }
  // Find relative pose from ref_frame_ to curr_frame_.
  if (!GeometrySolver::findRelativePoseRansac(ref_frame_, curr_frame_, F,
                                              inlier_matches_, T_curr_ref_,
                                              points_, triangulate_mask_)) {
    return false;
  }

  // If initialization succeeded, set poses accordingly.
  // Reference frame is fixed as world frame.
  ref_frame_->setPose(SE3(Mat33::Identity(), Vec3::Zero()));
  curr_frame_->setPose(T_curr_ref_ * ref_frame_->pose());
  return true;
}

bool Initializer::buildInitMap() {
  // Insert initial keyframes.
  ref_frame_->setKeyframe();
  curr_frame_->setKeyframe();
  tracker_->map_->insertKeyframe(ref_frame_);
  tracker_->map_->insertKeyframe(curr_frame_);

  // Insert initial map points.
  Frame::Features feats_1 = ref_frame_->feats_;
  Frame::Features feats_2 = curr_frame_->feats_;
  const int n_inlier_matches = inlier_matches_.size();
  for (int i = 0; i < n_inlier_matches; ++i) {
    if (!triangulate_mask_[i]) continue;
    // Create new map point.
    MapPoint::Ptr point = make_shared<MapPoint>(points_[i]);
    // Add Observations.
    Feature::Ptr feat_1 = feats_1[inlier_matches_[i].first];
    Feature::Ptr feat_2 = feats_2[inlier_matches_[i].second];
    point->addObservation(feat_1);
    point->addObservation(feat_2);
    // Link features with the map point.
    feat_1->point_ = point;
    feat_2->point_ = point;
    // Update map point characteristics.
    point->updateBestFeature();
    point->updateMedianViewDirAndScale();
    // Store the id of the frame where the map point is first observed by. Used
    // only for drawing purpose.
    point->ref_frame_id_ = ref_frame_->id_;
    // Insert to map the new created point.
    tracker_->map_->insertMapPoint(point);
  }

  // Update covisible information.
  ref_frame_->updateCoInfo();
  curr_frame_->updateCoInfo();

  // Global bundle adjustment to optimize poses and points' position jointly.
  Optimizer::globalBA(tracker_->map_);

  // FIXME What is the rescaling principle under the hood?
  // Rescale the map such that the mean scene depth is equal to 1.0
  const double scene_median_depth = ref_frame_->computeSceneMedianDepth();
  if (scene_median_depth < 0) {
    LOG(INFO) << "Negative scene_median_depth :" << scene_median_depth;
    return false;
  }
  const double scale_factor = 1. / scene_median_depth;
  // Scale the baseline between ref_frame_ and curr_frame_.
  SE3 T_c_w_curr = curr_frame_->pose();
  T_c_w_curr.translation() =
      -T_c_w_curr.rotationMatrix() *
      (ref_frame_->cam_->pos() +
       scale_factor * (curr_frame_->cam_->pos() - ref_frame_->cam_->pos()));
  curr_frame_->setPose(T_c_w_curr);
  // Scale the coordinates of map points.
  const list<MapPoint::Ptr>& points = tracker_->map_->getAllMapPoints();
  for (const auto& point : points) point->setPos(scale_factor * point->pos());

  return true;
}

void Initializer::setTracker(sptr<Tracking> tracker) {
  if (tracker_ == nullptr) tracker_ = tracker;
}

void Initializer::reset() {
  stage_ = Stage::NO_FRAME_YET;
  ref_frame_.reset();
  curr_frame_.reset();
  inlier_matches_.clear();
}

}  // namespace mono_slam