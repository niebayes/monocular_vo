#ifndef MONO_SLAM_BACK_END_LOCAL_MAPPING_H_
#define MONO_SLAM_BACK_END_LOCAL_MAPPING_H_

#include "mono_slam/common_include.h"
#include "mono_slam/front_end_tracking.h"
#include "mono_slam/keyframe_database.h"
#include "mono_slam/map.h"
#include "mono_slam/system.h"

namespace mono_slam {

class System;
class Tracking;
class Map;
class KeyframeDB;
class Frame;

class LocalMapping {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<LocalMapping>;
  using Keyframe = Frame;
  using Keyframes = std::queue<Keyframe::Ptr>;

  LocalMapping();

  void InsertKeyframe(const Keyframe::Ptr& keyframe);

  void Reset();

  // Setters to link components.
  void SetSystem(sptr<System> system);
  void SetTracker(sptr<Tracking> tracker);
  void SetMap(sptr<Map> map);
  void SetVocabulary(const sptr<Vocabulary>& voc);
  void SetKeyframeDB(KeyframeDB::Ptr keyframe_db);

 public:
  Keyframes keyframes_;

 private:
  std::mutex ownership_;

  sptr<System> system_ = nullptr;
  sptr<Tracking> tracker_ = nullptr;
  sptr<Map> map_ = nullptr;
  KeyframeDB::Ptr keyframe_db_ = nullptr;
  sptr<Vocabulary> voc_ = nullptr;
};

void InsertKeyframe(const Keyframe::Ptr& keyframe) {
  CHECK_EQ(Keyframe->IsKeyframe(), true);
  u_lock take(ownership_);
  keyframes_.push(keyframe);
}

void LocalMapping::Reset() {
  u_lock take(ownership_);
  while (!keyframes_.empty()) keyframes_.pop();
}

}  // namespace mono_slam

#endif  // MONO_SLAM_BACK_END_LOCAL_MAPPING_H_