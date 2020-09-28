#include "mono_slam/map.h"

namespace mono_slam {

// TODO(bayes) Use list data structure.
void Map::InsertKeyframe(Frame::Ptr keyframe) {
  CHECK_EQ(keyframe->IsKeyframe(), true);
  CHECK_GE(keyframe->id_, max_frame_id_);
  u_lock take(ownership_);
  keyframes_.push_back(keyframe);
}

void Map::EraseKeyframeById(const int id) {
  u_lock take(ownership_);
  // FIXME Efficiency issue? Use for loop and break out once found?
  keyframes_.remove_if(
      [](const Frame::Ptr& keyframe) { return keyframe->id_ == id; });
}

}  // namespace mono_slam