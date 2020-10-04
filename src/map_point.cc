#include "mono_slam/map_point.h"

namespace mono_slam {

MapPoint::MapPoint(const Vec3& pos)
    : id_(point_cnt_++), pos_(pos), is_outlier_(false) {}

// TODO(bayes) Compute mean_view_dirs_.
MapPoint::MapPoint(const Vec3& pos, const sptr<Feature>& feat)
    : id_(point_cnt_++),
      pos_(pos),
      best_feat_(feat),
      median_view_dir_(Vec3{}),
      is_outlier_(false) {
  observations_.push_back(feat);
}

void MapPoint::setPos(const Vec3& pos) {
  u_lock take(ownership_);
  pos_ = pos;
}

void MapPoint::addObservation(Feature::Ptr feat) {
  u_lock lock(mutex_);
  observations_.push_back(feat);
}

void MapPoint::eraseObservation(const Feature::Ptr& feat) {
  u_lock lock(mutex_);
  for (auto it = observations_.begin(), it_end = observations_.end();
       it != it_end; ++it) {
    if (*it == feat) {
      observations_.erase(it);
      feat->point_.reset();
      break;  // FIXME Should we break out immediately?
    }
  }
}

void MapPoint::updateBestFeature() {
  if (to_be_deleted_) return;
  // Container of all available features.
  vector<sptr<Feature>> feats;
  feats.reserve(this->nObs());
  for (const auto& feat_ : observations_) {
    if (feat_.expired()) continue;
    feats.push_back(feat_.lock());
  }

  const int num_feats = feats.size();
  vector<vector<int>> dists(num_feats, vector<int>(num_feats));
  // Compute pairwise distances row by row.
  for (int i = 0; i < num_feats; ++i) {  // row i.
    dists[i][i] = 0;
    // Computing upper triangular suffices.
    for (int j = i + 1; j < num_feats; ++j) {  // col j.
      const int dist = matcher_utils::computeDescriptorDistance(
          feats[i]->descriptor_, feats[j]->descriptor_);
      dists[i][j] = dist;
      dists[j][i] = dist;
    }
  }

  // Obtain the best feature which has the least median distance with others.
  int least_median_dist = 256;
  int idx = 0;
  for (int i = 0; i < num_feats; ++i) {
    vector<int>& dists_row_i(dists[i].begin(), dists[i].end());
    const int median_dist = math_utils::get_median(dists_row_i);
    if (median_dist < least_median_dist) {
      least_median_dist = median_dist;
      idx = i;
    }
  }

  best_feat_ = feats[idx];
}

void MapPoint::updateMedianViewDirAndScale() {
  u_lock lock(mutex_);
  vector<Vec3> view_dirs;  // Container for viewing directions.
  vector<int> levels;      // Container for viewing scales (aka. levels).
  for (const auto& feat_ : observations_) {
    if (feat_.expired()) continue;
    const auto& feat = feat_.lock();
    if (feat->frame_.expired()) continue;
    const auto& frame = feat->frame_.lock();
    const Vec3 unit_bear_vec = frame->cam_->getUnitBearVec(this->pos());
    view_dirs.push_back(unit_bear_vec);
    levels.push_back(feat->level_);
  }

  // Obtain median.
  median_view_dir_ = math_utils::get_median(view_dirs);
  median_view_scale_ = math_utils::get_median(levels);
}

bool MapPoint::isObservedBy(const sptr<Frame>& keyframe) const {
  CHECK_EQ(keyframe->isKeyframe(), true);
  u_lock lock(mutex_);
  for (auto it = observations_.begin(), it_end = observations_.end();
       it != it_end; ++it) {
    if ((*it)->frame_.lock() == keyframe) return true;
  }
  return false;
}

}  // namespace mono_slam
