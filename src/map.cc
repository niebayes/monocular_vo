#include "mono_slam/map.h"

namespace mono_slam {

void Map::InsertKeyframe(Frame::Ptr keyframe) {
  CHECK_EQ(keyframe->IsKeyframe(), true);
  u_lock take(ownership_);
  if (keyframes_.count(keyframe->id_))
    return;
  else
    keyframes_.insert(make_pair(keyframe->id_, keyframe));
}

void Map::InsertMapPoint(MapPoint::Ptr point) {
  CHECK_NE(point->IsOutlier(), true);
  u_lock take(ownership_);
  if (points_.count(point->id_))
    return;
  else
    points_.insert(make_pair(point->id_, point));
}

void Map::EraseKeyframeById(const int id) {
  u_lock take(ownership_);
  keyframes_.erase(id);
}

void Map::EraseMapPointById(const int id) {
  u_lock take(ownership_);
  points_.erase(id);
}

}  // namespace mono_slam