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
  extractFeatures();
  computeBoW();
  trackCurrentFrame();
  // This could only happen when relocalization was failed just now.
  if (curr_frame_ == nullptr) return;
  // If the system was just initialized, last_frame_ is selected as the datum
  // frame.
  if (datum_frame_) {
    last_frame_ = datum_frame_;
    last_frame_->is_datum_ = true;  // Datun frame won't be optimized out.
    datum_frame_.reset();
  }
  // Update constant velocity model, aka. relative motion.
  if (last_frame_)
    T_curr_last_ = curr_frame_->pose() * last_frame_->pose().inverse();
  viewer_->informUpdate();  // Update viewer.
  // Since viewer is racing the last_frame_ and curr_frame_, a lock is
  // employed to protect the shared data.
  lock_g lock(mut_);
  last_frame_ = curr_frame_;  // Update last frame.
  curr_frame_.reset();        // Reseat pointer making it ready for next frame.
  //! Decrease the reference counter once an object doesn't own it any more.
}

void Tracking::trackCurrentFrame() {
  switch (state_) {
    case State::NOT_INITIALIZED_YET:
      initializer_->setTracker(shared_from_this());
      if (initMap()) {
        last_kf_id_ = curr_frame_->id_;
        datum_frame_ = initializer_->ref_frame_;
        // local_mapper_->insertKeyframe(last_frame_);
        // local_mapper_->insertKeyframe(curr_frame_);
        // FIXME Should I inform local_mapper_ right now?
        // Asure that the update is performed after both the keyframes are
        // inserted. local_mapper_->informUpdate();
        state_ = State::GOOD;
      } else
        datum_frame_.reset();
      break;

    case State::GOOD:
      if (!trackFromLastFrame() || !trackFromLocalMap())
        state_ = State::LOST;
      else {
        if (needNewKf()) {
          last_kf_id_ = curr_frame_->id_;
          curr_frame_->setKeyframe();
          local_mapper_->insertKeyframe(curr_frame_);
          local_mapper_->informUpdate();
        }
      }
      break;

    case State::LOST:
      if (relocalization()) {
        last_kf_id_ = curr_frame_->id_;
        curr_frame_->setKeyframe();
        local_mapper_->insertKeyframe(curr_frame_);
        local_mapper_->informUpdate();
        state_ = State::GOOD;
      } else
        system_->reset();
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
  LOG(INFO) << "trackFromLastFrame ...";
  // Set initial pose in accordance with constant velocity model.
  curr_frame_->setPose(T_curr_last_ * last_frame_->pose());
  // Search matches by projection (i.e. project map points observed by last
  // frame onto current frame and try matching them against features around
  // them).
  const int n_matches = Matcher::searchByProjection(last_frame_, curr_frame_);
  LOG(INFO) << "matches(last_frame_, curr_frame_) = " << n_matches;
  if (n_matches < Config::min_n_matches()) {
    LOG(INFO) << "trackFromLastFrame failed.";
    return false;
  }
  const int n_inlier_matches = Optimizer::optimizePose(curr_frame_);
  LOG(INFO) << cv::format("trackFromLastFrame(n_inlier_matches: %d).",
                          n_inlier_matches);
  if (n_inlier_matches < Config::min_n_inlier_matches()) {
    LOG(INFO) << "trackFromLastFrame failed.";
    return false;
  }
  LOG(INFO) << "trackFromLastFrame succeeded.";
  return true;
}

bool Tracking::trackFromLocalMap() {
  LOG(INFO) << "trackFromLocalMap ...";
  updateLocalCoKfs();
  const int n_matches = Matcher::searchByProjection(local_co_kfs_, curr_frame_);
  LOG(INFO) << "matches(local_co_kfs_, curr_frame_) = " << n_matches;
  if (n_matches < Config::min_n_matches()) {
    LOG(INFO) << "trackFromLocalMap failed.";
    return false;
  }
  const int n_inlier_matches = Optimizer::optimizePose(curr_frame_);
  LOG(INFO) << cv::format("trackFromLocalMap(n_inlier_matches: %d).",
                          n_inlier_matches);
  if (n_inlier_matches < Config::min_n_inlier_matches()) {
    LOG(INFO) << "trackFromLocalMap failed.";
    return false;
  }
  LOG(INFO) << "trackFromLocalMap succeeded.";
  return true;
}

void Tracking::updateLocalCoKfs() {
  LOG(INFO) << "updateLocalCoKfs ...";
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
    // FIXME How could this frame get co_kfs_ right now?
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

  // Only retain keyframes with the shared number of map points exceeding this
  // threshold.
  const int min_weight_thresh = Config::weight_factor() * max_weight;
  auto it_ = local_co_kfs_.begin();
  for (; it_ != local_co_kfs_.end(); ++it_)
    if (co_kf_weights[*it_] < min_weight_thresh) local_co_kfs_.erase(it_);
  LOG(INFO) << local_co_kfs_.size() << " local covisible keyframes selected.";
}

bool Tracking::needNewKf() {
  return true;
  LOG(INFO) << "Need new keyframe?";
  bool need_new_kf = true;
  // TODO(bayes) Use a more complicated strategy.
  const int n_kfs = map_->nKfs();
  LOG(INFO) << n_kfs << " keyframes are in map right now.";
  // Cannot exceed maximal number of keyframes in map at one moment.
  if (n_kfs > Config::max_n_kfs_in_map()) need_new_kf = false;
  // Frequently creating new keyframe is forbidden.
  if (curr_frame_->id_ < last_kf_id_ + Config::new_kf_interval())
    need_new_kf = false;
  // FIXME Do we need to check this?
  // Local mapper cannot be busy.
  // if (!local_mapper_->isIdle()) need_new_kf = false;

  if (need_new_kf)
    LOG(INFO) << "New keyframe selected.";
  else
    LOG(INFO) << "Abandon this frame.";
  return need_new_kf;
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
    const int n_matches = Matcher::searchByBoW(kf, curr_frame_, matches);
    LOG(INFO) << "Reloc: matches(kf, curr_frame_) = " << n_matches << '\n';
    if (n_matches <= Config::reloc_min_n_matches()) continue;
    SE3 relative_pose;  // Relative pose from keyframe to current frame.
    if (!GeometrySolver::P3PRansac(kf, curr_frame_, matches, relative_pose)) {
      LOG(INFO) << "Reloc: failed to find relative pose.";
      continue;
    }
    curr_frame_->setPose(relative_pose);
    // Utilize pose graph optimization to count number of inliers.
    reloc_success = true;
    const int n_inlier_matches = Optimizer::optimizePose(curr_frame_);
    if (n_inlier_matches < Config::reloc_min_n_inlier_matches()) {
      reloc_success = true;
      break;  // Get out from loop once a acceptable candidate is found.
    }
  }
  if (reloc_success)
    LOG(INFO) << "Relocalization succeeded.";
  else
    LOG(INFO) << "Relocalization failed.";
  return reloc_success;
}

void Tracking::reset() {
  state_ = State::NOT_INITIALIZED_YET;
  initializer_.reset(new Initializer);
  last_frame_.reset();
  curr_frame_.reset();
  T_curr_last_ = SE3();
  local_co_kfs_.clear();
  // last_kf_id_ = 0;
}

void Tracking::extractFeatures() {
  vector<cv::KeyPoint> kpts;
  cv::Mat descriptors;
  detector_->detectAndCompute(curr_frame_->img_, cv::noArray(), kpts,
                              descriptors);
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

void Tracking::setSystem(sptr<System> system) { system_ = system; }
void Tracking::setLocalMapper(sptr<LocalMapping> local_mapper) {
  local_mapper_ = local_mapper;
}
void Tracking::setMap(Map::Ptr map) { map_ = map; }
void Tracking::setViewer(sptr<Viewer> viewer) { viewer_ = viewer; }

}  // namespace mono_slam
