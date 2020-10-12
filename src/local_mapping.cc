#include "mono_slam/local_mapping.h"

#include "mono_slam/g2o_optimizer.h"
#include "mono_slam/geometry_solver.h"
#include "mono_slam/matcher.h"
#include "mono_slam/utils/math_utils.h"

namespace mono_slam {

LocalMapping::LocalMapping() : is_idle_(true) { startThread(); }

void LocalMapping::startThread() {
  is_running_.store(true);
  // Spawn a new thread for local mapping.
  thread_ = std::thread(std::bind(&LocalMapping::LocalMappingLoop, this));
}

//! Not used currently.
void LocalMapping::stopThread() {
  LOG(INFO) << "Request stopping local mapper ...";
  is_running_.store(false);
  new_kf_cond_var_.notify_one();  // Release the lock or the halt won't proceed.
  thread_.join();
  LOG(INFO) << "Local mapper stopped.";
}

void LocalMapping::insertKeyframe(Frame::Ptr keyframe) {
  LOG(INFO) << "Try inserting keyframe " << keyframe->id_
            << " to local mapper ...";
  u_lock lock(mutex_);
  kfs_queue_.push(keyframe);
  LOG(INFO) << "Inserted keyframe " << keyframe->id_ << " to local mapper.";
}

void LocalMapping::informUpdate() {
  // Racing the mutex or the notification may not be performed.
  u_lock lock(mutex_);
  new_kf_cond_var_.notify_one();
}

void LocalMapping::LocalMappingLoop() {
  while (is_running_.load()) {
    {  // Don't hold lock to do time-consuming workload.
      u_lock lock(mutex_);
      new_kf_cond_var_.wait(lock);
      // FIXME Simply accumulate unprocessed keyframes? Perhaps a method
      // checkNewKfs() is applicable for this purpose.
      if (kfs_queue_.empty()) continue;
      is_idle_ = false;  // Don't let tracker interupt me!
      curr_keyframe_ = kfs_queue_.front();
      kfs_queue_.pop();
    }
    LOG(INFO) << "Local mapper is processing keyframe " << curr_keyframe_->id_;
    processFrontKeyframe();
    triangulateNewPoints();
    // Run local BA if keyframe queue is empty at this momment and the map
    // is maintaining more than 2 keyframes as well.
    if (kfs_queue_.empty() && map_->nKfs() > 2)
      Optimizer::localBA(curr_keyframe_, map_);
    removeRedundantKfs();
    LOG(INFO) << "Local mapper finished processing keyframe "
              << curr_keyframe_->id_;
    curr_keyframe_.reset();  // Always reseat shared_ptr once we don't need it.
    is_idle_ = true;
  }
}

void LocalMapping::processFrontKeyframe() {
  // Update links between current keyframe and map points.
  for (const Feature::Ptr& feat : curr_keyframe_->feats_) {
    const MapPoint::Ptr& point = feat_utils::getPoint(feat);
    if (!point || !point->isObservedBy(curr_keyframe_)) continue;
    point->addObservation(feat);
    point->updateBestFeature();
    point->updateMedianViewDirAndScale();
  }
  // Update covisibility information.
  curr_keyframe_->updateCoInfo();
  // Insert to map the new keyframe.
  map_->insertKeyframe(curr_keyframe_);
  LOG(INFO) << "Map now has " << map_->nKfs() << " keyframes.";
}

void LocalMapping::triangulateNewPoints() {
  // Get top 10 covisible keyframes ranked with number of shared map points.
  const forward_list<Frame::Ptr>& co_kfs = curr_keyframe_->getCoKfs(10);

  // Iterate all covisible keyframes.
  int n_new_points = 0;
  for (const Frame::Ptr& kf : co_kfs) {
    // Test if this keyframe and current keyframe under processing are able to
    // triangulate new good points.

    // Test 1: suffcient baseline.
    const Vec3 baseline =
        kf->cam_->getCamCenter() - curr_keyframe_->cam_->getCamCenter();
    const double median_depth = kf->computeSceneMedianDepth();
    // FIXME Magic 0.01?
    if (baseline.norm() / median_depth < 0.01) continue;

    // Search for putative matches.
    vector<int> matches;
    const int n_matches =
        Matcher::searchForTriangulation(kf, curr_keyframe_, matches);
    if (n_matches < Config::tri_min_n_matches()) continue;

    // Iterate all matches;
    for (int i = 0, i_end = matches.size(); i < i_end; ++i) {
      if (matches[i] == -1) continue;  // Skip unmatched features.
      // Test 2: sufficient parallax.
      const Feature::Ptr &feat_1 = kf->feats_[i],
                         &feat_2 = curr_keyframe_->feats_[matches[i]];
      const Vec2 &pt_1 = feat_1->pt_, &pt_2 = feat_2->pt_;
      const Vec3 bear_vec_1 = kf->cam_->pixel2bear(pt_1),
                 bear_vec_2 = curr_keyframe_->cam_->pixel2bear(pt_2);
      const double cos_parallax =
          bear_vec_1.dot(bear_vec_2) / (bear_vec_1.norm() * bear_vec_2.norm());
      if (cos_parallax <
          std::cos(math_utils::degree2radian(Config::tri_min_parallax())))
        continue;

      // Triangulate new point.
      const Mat33 &K_1 = kf->cam_->K(), &K_2 = curr_keyframe_->cam_->K();
      const Mat34 M_1 = K_1.inverse() * kf->pose().matrix3x4(),
                  M_2 = K_2.inverse() * curr_keyframe_->pose().matrix3x4();
      Vec3 point_1;  // Point in camera frame.
      geometry::triangulateLin(pt_1, pt_2, M_1, M_2, point_1);

      // Test 3: triangulated point is not infinitely far away as "infinite"
      // points can easily go to negative depth.
      if (point_1.array().isInf().any()) continue;
      // Test 4: triangulated point must have positive depth (in both cameras).
      const Vec3 point_2 = curr_keyframe_->pose() * point_1;
      if (point_1(2) <= 0. || point_2(2) <= 0.) continue;
      // Test 5: the reprojection error must below the tolerance.
      const double repr_err_1 = geometry::computeReprErr(point_1, pt_1, K_1),
                   repr_err_2 = geometry::computeReprErr(point_2, pt_2, K_2);
      const double chi2_thresh = 5.991;  // Two-degree chi-square p-value.
      const int level_1 = feat_1->level_, level_2 = feat_2->level_;
      if (repr_err_1 > Config::scale_level_sigma2().at(level_1) * chi2_thresh ||
          repr_err_2 > Config::scale_level_sigma2().at(level_2) * chi2_thresh)
        continue;
      // Test 6: scale consistency (i.e. the scale ratio and the distance ratio
      // should be in a close range).
      const double scale_ratio = Config::scale_factors().at(level_2) /
                                 Config::scale_factors().at(level_1);
      // point_1 is triangulated in world frame (actually, the frame of
      // keyframe).
      const double dist_1 = kf->cam_->getDistToCenter(point_1),
                   dist_2 = curr_keyframe_->cam_->getDistToCenter(point_1);
      if (dist_1 == 0) continue;  // Avoid dividing by zero.
      const double dist_ratio = dist_2 / dist_1;
      // Magic numbers whatsoever!
      if (scale_ratio / dist_ratio > 1.50 || scale_ratio / dist_ratio < 0.50)
        continue;

      // Create new map point if all tests are passed.
      MapPoint::Ptr point = make_shared<MapPoint>(point_1);
      point->addObservation(feat_1);
      point->addObservation(feat_2);
      // Update observation information.
      point->updateBestFeature();
      point->updateMedianViewDirAndScale();
      // Add to map the new created map point.
      map_->insertMapPoint(point);
      ++n_new_points;
    }
  }
  LOG(INFO) << "Triangulated " << n_new_points << " new map points.";
  LOG(INFO) << "Map now has " << map_->nPoints() << " map points.";
}

void LocalMapping::removeRedundantKfs() {
  // Iterate all covisible keyframes.
  const forward_list<Frame::Ptr>& co_kfs = curr_keyframe_->co_kfs_;
  int n_redun_kfs = 0;
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
        if (!kf || kf == kf_) continue;  // Self is of course excluded.
        // Features must be detected in neighbor scales.
        if (feat->level_ >= feat_->level_ - 1 &&
            feat->level_ <= feat_->level_ + 1)
          ++n_obs;
        if (n_obs >= n_obs_thresh) break;
      }
      if (n_obs >= n_obs_thresh) ++n_redundant_obs;
    }

    if (n_redundant_obs >= Config::redun_factor() * n_points) {
      map_->removeKeyframe(kf_);  // Remove redundant keyframe from map.
      ++n_redun_kfs;
    }
  }
  LOG(INFO) << "Removed " << n_redun_kfs << " redundant keyframes.";
}

void LocalMapping::reset() {
  u_lock lock(mutex_);
  while (!kfs_queue_.empty()) kfs_queue_.pop();
  curr_keyframe_.reset();
}

void LocalMapping::setSystem(sptr<System> system) { system_ = system; }
void LocalMapping::setTracker(sptr<Tracking> tracker) { tracker_ = tracker; }
void LocalMapping::setMap(sptr<Map> map) { map_ = map; }

}  // namespace mono_slam
