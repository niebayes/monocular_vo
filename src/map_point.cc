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
  observations_.push_front(feat);
}

void SetPos(const Vec3& pos) {
  u_lock take(ownership_);
  pos_ = pos;
}

void MapPoint::AddObservation(const sptr<Feature>& feat) {
  u_lock take(ownership_);
  observations_.insert(feat);
}

void MapPoint::EraseObservation(const sptr<Feature>& feat) {
  u_lock take(ownership_);
  observations_.erase(feat);
}

void MapPoint::UpdateBestDescriptor() {
  //
}

void MapPoint::UpdateMeanViewingDirection() {
  //
}


}  // namespace mono_slam
