#include "mono_slam/map_point.h"

#include "mono_slam/feature.h"
#include "mono_slam/frame.h"
#include "mono_slam/matcher.h"
#include "mono_slam/utils/math_utils.h"

namespace mono_slam {

int MapPoint::point_cnt_ = 0;

MapPoint::MapPoint(const Vec3& pos) : id_(point_cnt_++), pos_(pos) {}

// TODO(bayes) Compute mean_view_dirs_.
MapPoint::MapPoint(const Vec3& pos, sptr<Feature> feat)
    : id_(point_cnt_++), pos_(pos), best_feat_(feat) {
  observations_.push_back(feat);
}

void MapPoint::setPos(const Vec3& pos) {
  u_lock lock(mutex_);
  pos_ = pos;
}

void MapPoint::addObservation(sptr<Feature> feat) {
  u_lock lock(mutex_);
  observations_.push_back(feat);
}

void MapPoint::eraseObservation(const sptr<Feature>& feat) {
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
  for (const sptr<Feature>& feat : observations_) {
    feats.push_back(feat);
  }

  const int num_feats = feats.size();
  vector<vector<int>> dists(num_feats, vector<int>(num_feats));
  // Compute pairwise distances row by row.
  for (int i = 0; i < num_feats; ++i) {  // row i.
    dists[i][i] = 0;
    // Computing upper triangular suffices.
    for (int j = i + 1; j < num_feats; ++j) {  // col j.
      const int dist = matcher_utils::computeDescDist(feats[i]->descriptor_,
                                                      feats[j]->descriptor_);
      dists[i][j] = dist;
      dists[j][i] = dist;
    }
  }

  // Obtain the best feature which has the least median distance with others.
  int least_median_dist = 256;
  int idx = 0;
  for (int i = 0; i < num_feats; ++i) {
    vector<int> dists_row_i(dists[i].begin(), dists[i].end());
    const int median_dist = math_utils::get_median(dists_row_i);
    if (median_dist < least_median_dist) {
      least_median_dist = median_dist;
      idx = i;
    }
  }

  best_feat_ = feats[idx];
}

void MapPoint::updateMedianViewDirAndScale() {
  const Vec3 pos = this->pos();
  u_lock lock(mutex_);
  Vec3 total_view_dir;  // Container for viewing directions.
  vector<int> levels;   // Container for viewing scales (aka. levels).
  int n = 0;
  for (const sptr<Feature>& feat : observations_) {
    if (feat->frame_.expired()) continue;
    const auto& frame = feat->frame_.lock();
    const Vec3 unit_bear_vec = frame->cam_->getUnitBearVec(pos);
    total_view_dir += unit_bear_vec;
    levels.push_back(feat->level_);
    ++n;
  }

  // Obtain median.
  //! Since the median of circular data is not well-defined, we simply use mean
  //! to replace it. Hopefully, we will solve it soon.
  median_view_dir_ = total_view_dir / n;
  median_view_scale_ = math_utils::get_median(levels);
}

bool MapPoint::isObservedBy(const sptr<Frame>& keyframe) const {
  CHECK_EQ(keyframe->isKeyframe(), true);
  u_lock lock(mutex_);
  for (const sptr<Feature>& feat : observations_)
    if (feat->frame_.lock() == keyframe) return true;
  return false;
}

}  // namespace mono_slam
