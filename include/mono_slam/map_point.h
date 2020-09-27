#ifndef MONO_SLAM_MAP_POINT_H_
#define MONO_SLAM_MAP_POINT_H_

#include "mono_slam/common_include.h"
#include "mono_slam/feature.h"
#include "mono_slam/frame.h"

namespace mono_slam {

class Feature;
class Frame;
class MapPoint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = sptr<MapPoint>;
  using Keyframe = Frame;

  MapPoint(const Vec3& pos);
  MapPoint(const Vec3& pos, const wptr<Feature>& feat);

  inline const Vec3& Pos() const {
    u_lock take(ownership_);
    return pos_;
  }

  void SetPos(const Vec3& pos);

  inline bool IsOutlier() const {
    u_lock take(ownership_);
    return is_outlier_;
  }

  void SetOutlier() {
    u_lock take(ownership_);
    is_outlier_ = true;
  }

  // Add an observation.
  void AddObservation(const wptr<Feature>& feat);

  // Erase an observation.
  void EraseObservation(const wptr<Feature>& feat);

  inline const list<Feature::Ptr>& GetAllObservations() const {
    u_lock take(ownsership_);
    return observations_;
  }

  inline int NumObservations() const {
    u_lock take(ownsership_);
    return observations_.size();
  }

  void UpdateBestDescriptor();

  void UpdateMeanViewingDirection();

  // Check if this map point is observed by the given keyframe.
  inline bool IsObservedBy(const sptr<Keyframe>& keyframe) const;

 public:
  // Observations.
  list<wptr<Feature>> observations_;

  // Map point characteristics.
  static int point_cnt_;  // Global map point counter, starting from 0.
  const int id_;          // Unique map point identity.
  Vec3 pos_;              // Position in world frame.
  wptr<Feature> best_feat_ =
      nullptr;  // Best feature in that its descriptor has the least median
                // distance among all observations.
  Vec3 mean_view_dir_;  // Mean viewing direction.
  bool is_outlier_;     // Is this map point an outlier marked in optimization?

 private:
  std::mutex ownership_;
};

MapPoint::MapPoint(const Vec3& pos)
    : id_(point_cnt_++), pos_(pos), is_outlier_(false) {}

// TODO(bayes) Compute mean_view_dirs_.
MapPoint::MapPoint(const Vec3& pos, const wptr<Feature>& feat)
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

void MapPoint::AddObservation(const wptr<Feature>& feat) {
  u_lock take(ownership_);
  observations_.insert(feat);
}

void MapPoint::EraseObservation(const wptr<Feature>& feat) {
  u_lock take(ownership_);
  observations_.erase(feat);
}

void MapPoint::UpdateBestDescriptor() {
  //
}

void MapPoint::UpdateMeanViewingDirection() {
  //
}

inline bool IsObservedBy(const sptr<Keyframe>& keyframe) const {
  CHECK_EQ(keyframe->IsKeyframe(), true);
  u_lock take(ownership_);
  for (auto it = observations_.cbegin(), it_end = observations_.cend();
       it != it_end; ++it)
    // FIXME Is this equal operation valid? Should I overload a "==" operator?
    if ((*it)->frame_ == keyframe) return true;
  return false;
}

}  // namespace mono_slam

#endif  // MONO_SLAM_MAP_POINT_H_