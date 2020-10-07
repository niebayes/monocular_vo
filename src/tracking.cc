#include "mono_slam/tracking.h"

#include "mono_slam/g2o_optimizer.h"
#include "mono_slam/geometry_solver.h"
#include "mono_slam/matcher.h"


namespace mono_slam {

class Optimizer;

Tracking::Tracking() : state_(State::NOT_INITIALIZED_YET) {
  initializer_.reset(new Initializer());
  detector_ = cv::ORB::create(Config::max_n_feats());
}

void Tracking::addImage(const cv::Mat& img) {
  // Create a new frame and preprocess it.
  curr_frame_.reset(new Frame(img));
  extractFeatures(img);
#ifndef DEBUG
  computeBoW();
#endif

  trackCurrentFrame();
  last_frame_ = curr_frame_;  // Update last frame.
  // Update constant velocity model, aka. relative motion.
  T_curr_last_ = curr_frame_->pose() * last_frame_->pose().inverse();
  curr_frame_.reset();  // Reseat pointer making it ready for next frame.
  //! Actually, this makes no sense.
}

void Tracking::extractFeatures(const cv::Mat& img) {
  vector<cv::KeyPoint> kpts;
  cv::Mat descriptors;
  detector_->detectAndCompute(img, cv::noArray(), kpts, descriptors);
  if (curr_frame_->cam_->distCoeffs()(0) != 0)  // If having distortion.
    frame_utils::undistortKeypoints(curr_frame_->cam_->K(),
                                    curr_frame_->cam_->distCoeffs(), kpts);
  const int n_kpts = kpts.size();
  curr_frame_->feats_.reserve(n_kpts);
  for (int i = 0; i < n_kpts; ++i) {
    curr_frame_->feats_.push_back(
        make_shared<Feature>(curr_frame_, Vec2{kpts[i].pt.x, kpts[i].pt.y},
                             descriptors.row(i), kpts[i].octave));
  }
}

void Tracking::computeBoW() {
  // Collect descriptors into vector as DBoW's command.
  vector<cv::Mat> descriptor_vec;
  descriptor_vec.reserve(curr_frame_->nObs());
  std::transform(curr_frame_->feats_.cbegin(), curr_frame_->feats_.cend(),
                 std::back_inserter(descriptor_vec),
                 [](const Feature::Ptr& feat) { return feat->descriptor_; });
  voc_->transform(descriptor_vec, curr_frame_->bow_vec_, curr_frame_->feat_vec_,
                  4);
}

void Tracking::trackCurrentFrame() {
  switch (state_) {
    case State::NOT_INITIALIZED_YET:
      initializer_->setTracker(shared_from_this());
      if (initMap()) {
        last_kf_id_ = curr_frame_->id_;
        local_mapper_->insertKeyframe(last_frame_);
        local_mapper_->insertKeyframe(curr_frame_);
        state_ = State::GOOD;
      }
      break;

    case State::GOOD:
      if (!trackFromLastFrame() || !trackFromLocalMap())
        state_ = State::LOST;
      else {
        if (needNewKf()) {
          last_kf_id_ = curr_frame_->id_;
          curr_frame_->setKeyframe();
          local_mapper_->insertKeyframe(curr_frame_);
        }
      }
      break;

    case State::LOST:
      if (relocalization()) {
        last_kf_id_ = curr_frame_->id_;
        curr_frame_->setKeyframe();
        local_mapper_->insertKeyframe(curr_frame_);
        state_ = State::GOOD;
      } else
        reset();
      break;
  }
}

bool Tracking::initMap() {
  if (initializer_->stage() == Stage::NO_FRAME_YET)
    initializer_->addReferenceFrame(curr_frame_);
  else if (initializer_->stage() == Stage::HAS_REFERENCE_FRAME)
    initializer_->addCurrentFrame(curr_frame_);
  return initializer_->stage() == Stage::SUCCESS;
}

bool Tracking::trackFromLastFrame() {
  // Set initial pose in accordance with constant velocity model.
  curr_frame_->setPose(T_curr_last_ * last_frame_->pose());
  // Search matches by projection (i.e. project map points observed by last
  // frame onto current frame and try matching them against features around
  // them).
  const int num_matches = Matcher::searchByProjection(last_frame_, curr_frame_);
  if (num_matches < Config::min_n_matches()) return false;
  const int num_inlier_matches = Optimizer::optimizePose(curr_frame_);
  if (num_inlier_matches < Config::min_n_inlier_matches()) return false;
  return true;
}

bool Tracking::trackFromLocalMap() {
  updateLocalCoKfs();
  const int num_matches =
      Matcher::searchByProjection(local_co_kfs_, curr_frame_);
  if (num_matches < Config::min_n_matches()) return false;
  const int num_inlier_matches = Optimizer::optimizePose(curr_frame_);
  if (num_inlier_matches < Config::min_n_inlier_matches()) return false;
  return true;
}

void Tracking::updateLocalCoKfs() {
  local_co_kfs_.clear();  // Clear it first.

  // Covisible keyframe voter with the key being the covisible keyframe and the
  // weights being the number of shared map points.
  unordered_map<Frame::Ptr, int> co_kf_weights;
  // co_kf_weights.reserve(?). //! Don't know how much capacity to be reserved.

  // Obtain all covisible keyframes (i.e. ones sharing at least one map point).
  for (const Feature::Ptr& feat_ : curr_frame_->feats_) {
    const MapPoint::Ptr& point = feat_utils::getPoint(feat_);
    if (!point) continue;
    for (const Feature::Ptr& feat : point->getObservations()) {
      const Frame::Ptr& kf = feat_utils::getKeyframe(feat);
      if (!kf || kf == curr_frame_) continue;  // Self of course is excluded.
      co_kf_weights[kf]++;
      // Insert to local covisible keyframes.
      local_co_kfs_.insert(kf);  // Using set precludes repeat insertion.
    }
  }
  if (local_co_kfs_.empty()) return;

  // Add the keyframes covisible with current members in the local covisible
  // keyframes.
  // FIXME Do we realy need to do this?
  //! Iterate the copy of the local_co_kfs since we need to insert new elements
  //! into it in progress.
  for (const Frame::Ptr& kf_ : unordered_set<Frame::Ptr>(local_co_kfs_)) {
    // Get top 10 keyframes ranked wrt. number of covisible map points.
    const forward_list<Frame::Ptr>& co_kfs = kf_->getCoKfs(10);
    if (co_kfs.empty()) continue;
    for (const Frame::Ptr& kf : co_kfs) {
      if (kf == curr_frame_) continue;
      co_kf_weights[kf]++;
      local_co_kfs_.insert(kf);
    }
  }

  // Obtain the maximal weight.
  int max_weight = 0;
  auto it = co_kf_weights.cbegin(), it_end = co_kf_weights.cend();
  for (; it != it_end; ++it)
    if (it->second > max_weight) max_weight = it->second;

  // Only retain keyframes with the shared number of map points exceed this
  // threshold.
  const int min_weight_thresh = Config::weight_factor() * max_weight;
  auto it_ = local_co_kfs_.begin();
  for (; it_ != local_co_kfs_.end(); ++it_)
    if (co_kf_weights[*it_] < min_weight_thresh) local_co_kfs_.erase(it_);
}

bool Tracking::needNewKf() {
  // TODO(bayes) Use a more complicated strategy.
  const int n_kfs = map_->nKfs();
  // Cannot exceed maximal number of keyframes in map at one moment.
  if (n_kfs > Config::max_n_kfs_in_map()) return false;
  // Frequently creating new keyframe is forbidden.
  if (curr_frame_->id_ < last_kf_id_ + Config::new_kf_interval()) return false;
  // Local mapper cannot be busy.
  if (!local_mapper_->isIdle()) return false;
  return true;
}

bool Tracking::relocalization() {
  // Obtain relocalization candidates.
  list<Frame::Ptr> candidate_kfs;
  if (!(map_->kf_db_->detectRelocCandidates(curr_frame_, candidate_kfs)))
    return false;
  // Iterate all candidates.
  bool reloc_success = false;
  for (const Frame::Ptr& kf : candidate_kfs) {
    // Matches from relocalization candidate keyframe to current frame such that
    // kf[i] = curr_frame_[matches[i]];
    vector<int> matches;
    const int num_matches = Matcher::searchByBoW(kf, curr_frame_, matches);
    if (num_matches <= Config::reloc_min_n_matches()) continue;
    SE3 relative_pose;  // Relative pose from keyframe to current frame.
    if (!GeometrySolver::P3PRansac(kf, curr_frame_, matches, relative_pose))
      continue;
    curr_frame_->setPose(relative_pose);
    // Utilize pose graph optimization to count number of inliers.
    const int num_inlier_matches = Optimizer::optimizePose(curr_frame_);
    if (num_inlier_matches < Config::reloc_min_n_inlier_matches()) {
      reloc_success = true;
      break;  // Get out from loop once a acceptable candidate is found.
    }
  }
  return reloc_success;
}

void Tracking::reset() {
  state_ = State::NOT_INITIALIZED_YET;
  last_frame_.reset();
  curr_frame_.reset();
  T_curr_last_ = SE3();
  local_co_kfs_.clear();
  last_kf_id_ = 0;
}

void Tracking::setSystem(sptr<System> system) { system_ = system; }
void Tracking::setLocalMapper(sptr<LocalMapping> local_mapper) {
  local_mapper_ = local_mapper;
}
void Tracking::setMap(Map::Ptr map) { map_ = map; }
void Tracking::setViewer(sptr<Viewer> viewer) { viewer_ = viewer; }

}  // namespace mono_slam
