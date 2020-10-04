#include "mono_slam/back_end_local_mapping.h"

namespace mono_slam {

LocalMapping::LocalMapping() {}

void LocalMapping::insertKeyframe(const Frame::Ptr& keyframe) {
  CHECK_EQ(keyframe->isKeyframe(), true);
  u_lock lock(mutex_);
  keyframes_.push(keyframe);
}

void LocalMapping::reset() {
  u_lock lock(mutex_);
  while (!keyframes_.empty()) keyframes_.pop();
}

void LocalMapping::setSystem(sptr<System> system) { system_ = system; }
void LocalMapping::setTracker(sptr<Tracking> tracker) { tracker_ = tracker; }
void LocalMapping::setMap(sptr<Map> map) { map_ = map; }
void LocalMapping::setVocabulary(const sptr<Vocabulary>& voc) { voc_ = voc; }
void LocalMapping::setKeyframeDB(KeyframeDB::Ptr keyframe_db) {
  keyframe_db_ = keyframe_db;
}

}  // namespace mono_slam
