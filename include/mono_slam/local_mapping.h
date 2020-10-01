#ifndef MONO_SLAM_BACK_END_LOCAL_MAPPING_H_
#define MONO_SLAM_BACK_END_LOCAL_MAPPING_H_

#include "mono_slam/common_include.h"
#include "mono_slam/frame.h"
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
  using Keyframes = std::queue<Frame::Ptr>;

  LocalMapping();

  void insertKeyframe(const Frame::Ptr& keyframe);

  void reset();

  // Setters to link components.
  void setSystem(sptr<System> system);
  void setTracker(sptr<Tracking> tracker);
  void setMap(sptr<Map> map);
  void setVocabulary(const sptr<Vocabulary>& voc);
  void setKeyframeDB(KeyframeDB::Ptr keyframe_db);

 private:
  Keyframes keyframes_;
  mutable std::mutex ownership_;

  sptr<System> system_ = nullptr;
  sptr<Tracking> tracker_ = nullptr;
  sptr<Map> map_ = nullptr;
  KeyframeDB::Ptr keyframe_db_ = nullptr;
  sptr<Vocabulary> voc_ = nullptr;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_BACK_END_LOCAL_MAPPING_H_