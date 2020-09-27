#include "mono_slam/map.h"

void InsertKeyframe(const Keyframe::Ptr& keyframe) {
  CHECK_EQ(Keyframe->IsKeyframe(), true);
  u_lock take(ownership_);
  if (keyframes.count(keyframe->id_))
    return;
  else
    keyframes.insert(make_pair(keyframe->id_, keyframe));
}

void InsertMapPoint(const MapPoint::Ptr& point) {
  CHECK_NE(point->IsOutlier(), true);
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
