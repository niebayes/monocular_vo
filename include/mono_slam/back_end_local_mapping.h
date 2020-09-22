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

class LocalMapping {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<LocalMapping>;

  LocalMapping();

  void Reset();

  // Setters to link components.
  void SetSystem(sptr<System> system);
  void SetTracker(sptr<Tracking> tracker);
  void SetMap(sptr<Map> map);
  void SetVocabulary(const sptr<Vocabulary>& voc);
  void SetKeyframeDB(KeyframeDB::Ptr keyframe_db);

 private:
  sptr<System> system_ = nullptr;
  sptr<Tracking> tracker_ = nullptr;
  sptr<Map> map_ = nullptr;
  KeyframeDB::Ptr keyframe_db_ = nullptr;
  sptr<Vocabulary> voc_ = nullptr;
};

void LocalMapping::Reset() {}

void LocalMapping::SetSystem(sptr<System> system) { system_ = system; }
void LocalMapping::SetTracker(sptr<Tracking> tracker) { tracker_ = tracker; }
void LocalMapping::SetMap(sptr<Map> map) { map_ = map; }
void LocalMapping::SetVocabulary(const sptr<Vocabulary>& voc) { voc_ = voc; }
void LocalMapping::SetKeyframeDB(KeyframeDB::Ptr keyframe_db) {
  keyframe_db_ = keyframe_db;
}

}  // namespace mono_slam

#endif  // MONO_SLAM_BACK_END_LOCAL_MAPPING_H_