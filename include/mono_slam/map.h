#ifndef MONO_SLAM_MAP_H_
#define MONO_SLAM_MAP_H_

#include "mono_slam/common_include.h"
#include "mono_slam/frame.h"
#include "mono_slam/map_point.h"

namespace mono_slam {

class Frame;
class MapPoint;

class Map {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = sptr<Map>;
  using Keyframe = Frame;
  using Keyframes = std::unordered_map<int, Keyframe::Ptr>;
  using MapPoints = std::unordered_map<int, MapPoint::Ptr>;

  void InsertKeyframe(const Keyframe::Ptr& keyframe);

  void InsertMapPoint(const MapPoint::Ptr& point);

  void EraseKeyframeById(const int id);

  void EraseMapPointById(const int id);

  inline list<Keyframe::Ptr> GetAllKeyframes {
    u_lock take(ownership_);
    list<Keyframe::Ptr> keyframes;
    for (auto& id_kf : keyframes_) keyframes.push_back(id_kf.second);
    return keyframes;
  }

  inline list<MapPoint::Ptr> GetAllMapPoints {
    u_lock take(ownership_);
    list<MapPoint::Ptr> points;
    points.reserve(points_.size());
    for (auto& id_point : points_) points.push_front(id_point.second);
    return points;
  }

  inline Keyframes GetAllKeyframesWithId {
    u_lock take(ownership_);
    return keyframes_;
  }

  inline MapPoints GetAllMapPointsWithId {
    u_lock take(ownership_);
    return points_;
  }

  inline void Clear() {
    u_lock take(ownership_);
    keyframes_.clear();
    points_.clear();
  }

 private:
  Keyframes keyframes_;  // Maintained keyframes.
  MapPoints points_;     // Maintained map points.

  mutable std::mutex ownership_;
};
}  // namespace mono_slam

#endif  // MONO_SLAM_MAP_H_