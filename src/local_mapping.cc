#include "mono_slam/back_end_local_mapping.h"

namespace mono_slam {

LocalMapping::LocalMapping() { startThread(); }

void LocalMapping::startThread() {
  is_idle_.store(false);
  thread_ = make_unique(std::thread(std::bind(&LocalMappingLoop, this)));
}

void LocalMapping::stopThread() {
  is_idle_.store(true);
  new_kf_cond_var_.notify_one();
  thread_->join();
}

void LocalMapping::LocalMappingLoop() {
  //
}

void LocalMapping::insertNewKf(Frame::Ptr keyframe) {
  CHECK_EQ(keyframe->isKeyframe(), true);
  u_lock lock(mutex_);
  kfs_queue_.push(keyframe);
}

void LocalMapping::processNewKf() {
  u_lock lock(mutex_);
  curr_keyframe_ = kfs_queue_.front();
  kfs_queue_.pop();

  // Update links between current keyframe and map points.
  for (const Feature::Ptr& feat : curr_keyframe_->feats_) {
    const MapPoint::Ptr& point = feat_utils::getPoint(feat);
    if (!point || point->isObservedBy(curr_keyframe_)) continue;
    point->addObservation(feat);
    point->updateBestFeature();
    point->updateMedianViewDirAndDepth();
  }

  // Update covisibility information.
  curr_keyframe_->updateCoInfo();

  // Insert to map the new keyframe.
  map->insertKeyframe(curr_keyframe_);
}

void triangulateNewPoints() {
  //
}

void removeRedundantKfs() {
  // Iterate all covisible keyframes.
  const forward_list<Frame::Ptr>& co_kfs = curr_keyframe_->co_kfs_;
  for (const Frame::Ptr& kf_ : co_kfs) {
    int n_points = 0;            // Number of effective map points.
    int n_redundant_obs = 0;     // Number of redundant observations;
    const int n_obs_thresh = 3;  // If more than three keyframes observing the
                                 // same map point, it's marked as redundant.
    // Iterate all linked map points of this keyframe.
    for (const Feature::Ptr& feat_ : kf_->feats_) {
      const MapPoint::Ptr& point = feat_utils::getPoint(feat_);
      if (!point) continue;
      ++n_points;

      // Iterate all observations of this map point.
      int n_obs = 0;  // Number of observations of this map point.
      const list<Feature::Ptr>& observations = point->getObservations();
      for (const Feature::Ptr& feat : observations) {
        const Frame::Ptr& kf = feat_utils::getKeyframe(feat);
        if (!kf || kf == kf_) continue;
        // Features must be detected in neighbor scales.
        if (feat->level_ >= feat_->level_ - 1 &&
            feat->level_ <= feat_->level_ + 1)
          ++n_obs;
        if (n_obs >= n_obs_thresh) break;
      }
      if (n_obs >= n_obs_thresh) ++n_redundant_obs;
    }

    if (n_redundant_obs >= Config::redun_point_factor() * n_points) {
      // TODO(bayes) Mark this keyframe to be deleted.
    }
  }
}

void LocalMapping::reset() {
  u_lock lock(mutex_);
  while (!kfs_queue_.empty()) kfs_queue_.pop();
}

void LocalMapping::setSystem(sptr<System> system) { system_ = system; }
void LocalMapping::setTracker(sptr<Tracking> tracker) { tracker_ = tracker; }
void LocalMapping::setMap(sptr<Map> map) { map_ = map; }
void LocalMapping::setVocabulary(const sptr<Vocabulary>& voc) { voc_ = voc; }
void LocalMapping::setKeyframeDB(KeyframeDB::Ptr keyframe_db) {
  keyframe_db_ = keyframe_db;
}

}  // namespace mono_slam
