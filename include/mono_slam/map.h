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

  void InsertKeyframe(Keyframe::Ptr keyframe);

  void InsertMapPoint(MapPoint::Ptr point);

  void EraseKeyframeById(const int id);

  void EraseMapPointById(const int id);

  inline const Keyframes& GetAllKeyframes const {
    u_lock take(ownership_);
    return keyframes_;
  }

  inline const MapPoints& GetAllMapPoints const {
    u_lock take(ownership_);
    return points_;
  }

  void Clear();

 private:
  Keyframes keyframes;  // Maintained keyframes.
  MapPoints points;     // Maintained map points.

  std::mutex ownership_;
};

void InsertKeyframe(Keyframe::Ptr keyframe) {
  u_lock take(ownership_);
  if (keyframes.count(keyframe->id_))
    return;
  else
    keyframes.insert(make_pair(keyframe->id_, keyframe));
}

void InsertMapPoint(MapPoint::Ptr point) {
  u_lock take(ownsership_);
  if (points.count(point->id_)) 
    return; 
  else 
    points.insert(make_pair(point->id_), point);
}

void EraseKeyframeById(const int id) {
  u_lock take(ownership_);
  keyframes.erase(id);
}

void EraseMapPointById(const int id) {
  u_lock take(ownership_);
  points.erase(id);
}

void Map::Clear() {
  u_lock take(ownership_);
  keyframes.clear();
  points.clear();
}

}  // namespace mono_slam

#endif  // MONO_SLAM_MAP_H_