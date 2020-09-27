#include "mono_slam/back_end_local_mapping.h"

namespace mono_slam {

LocalMapping::LocalMapping() {}

void LocalMapping::InsertKeyframe(const Frame::Ptr& keyframe) {
  CHECK_EQ(keyframe->IsKeyframe(), true);
  u_lock take(ownership_);
  keyframes_.push(keyframe);
}

void LocalMapping::Reset() {
  u_lock take(ownership_);
  while (!keyframes_.empty()) keyframes_.pop();
}

void LocalMapping::SetSystem(sptr<System> system) { system_ = system; }
void LocalMapping::SetTracker(sptr<Tracking> tracker) { tracker_ = tracker; }
void LocalMapping::SetMap(sptr<Map> map) { map_ = map; }
void LocalMapping::SetVocabulary(const sptr<Vocabulary>& voc) { voc_ = voc; }
void LocalMapping::SetKeyframeDB(KeyframeDB::Ptr keyframe_db) {
  keyframe_db_ = keyframe_db;
}

}  // namespace mono_slam
