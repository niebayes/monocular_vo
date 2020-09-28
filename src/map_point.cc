#include "mono_slam/map_point.h"

namespace mono_slam {

MapPoint::MapPoint(const Vec3& pos)
    : id_(point_cnt_++), pos_(pos), is_outlier_(false) {}

// TODO(bayes) Compute mean_view_dirs_.
MapPoint::MapPoint(const Vec3& pos, const sptr<Feature>& feat)
    : id_(point_cnt_++),
      pos_(pos),
      best_feat_(feat),
      mean_view_dir_(Vec3{}),
      is_outlier_(false) {
  observations_.push_back(feat);
}

void MapPoint::SetPos(const Vec3& pos) {
  u_lock take(ownership_);
  pos_ = pos;
}

void MapPoint::AddObservation(const sptr<Feature>& feat) {
  u_lock take(ownership_);
  observations_.push_back(feat);
}

void MapPoint::EraseObservation(sptr<Feature>& feat) {
  u_lock take(ownership_);
  for (auto it = observations_.begin(), it_end = observations_.end();
       it != it_end; ++it) {
    if (it->lock() == feat) {
      observations_.erase(it);
      feat->point_.reset();
    }
  }
}

void MapPoint::UpdateBestDescriptor() {
  //
}

void MapPoint::UpdateMeanViewingDirection() {
  //
}

bool MapPoint::IsObservedBy(const sptr<Frame>& keyframe) const {
  CHECK_EQ(keyframe->IsKeyframe(), true);
  u_lock take(ownership_);
  for (auto it = observations_.begin(), it_end = observations_.end();
       it != it_end; ++it) {
    // FIXME Any good way to polish the codes?
    if (it->lock()->frame_.lock() == keyframe) return true;
  }
  return false;
}

}  // namespace mono_slam
