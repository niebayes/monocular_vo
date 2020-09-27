#ifndef MONO_SLAM_MAP_POINT_H_
#define MONO_SLAM_MAP_POINT_H_

#include "mono_slam/common_include.h"
#include "mono_slam/feature.h"
#include "mono_slam/frame.h"

namespace mono_slam {

class Frame;
class Feature;

class MapPoint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = sptr<MapPoint>;

  // Observations.
  list<wptr<Feature>> observations_;

  // Map point characteristics.
  static int point_cnt_;     // Global map point counter, starting from 0.
  const int id_;             // Unique map point identity.
  Vec3 pos_;                 // Position in world frame.
  wptr<Feature> best_feat_;  // Best feature in that its descriptor has the
                             // least median distance among all observations.
  Vec3 mean_view_dir_;       // Mean viewing direction.
  bool is_outlier_;  // Is this map point an outlier marked in optimization?

  MapPoint(const Vec3& pos);

  MapPoint(const Vec3& pos, const sptr<Feature>& feat);

  inline const Vec3& Pos() const {
    u_lock lock(ownership_);
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
  void AddObservation(const sptr<Feature>& feat);

  // Erase an observation.
  void EraseObservation(const sptr<Feature>& feat);

  inline list<sptr<Feature>> GetAllObservations() const {
    u_lock take(ownership_);
    list<sptr<Feature>> observations;
    for (auto feat : observations_) observations.push_back(feat.lock());
    return observations;
  }

  inline int NumObs() const {
    u_lock take(ownership_);
    return observations_.size();
  }

  void UpdateBestDescriptor();

  void UpdateMeanViewingDirection();

  // Check if this map point is observed by the given keyframe.
  inline bool IsObservedBy(const sptr<Frame>& keyframe) const {
    CHECK_EQ(keyframe->IsKeyframe(), true);
    u_lock take(ownership_);
    for (auto it = observations_.cbegin(), it_end = observations_.cend();
         it != it_end; ++it)
      // FIXME Is this equal operation valid? Should I overload a "==" operator?
      if (it->lock()->frame_ == keyframe) return true;
    return false;
  }

 private:
  mutable std::mutex ownership_;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_MAP_POINT_H_