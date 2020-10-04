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

  LocalMapping();

  void startThread();

  void stopThread();

  void insertKeyframe(Frame::Ptr keyframe);

  void LocalMappingLoop();

  void processFrontKeyframe();

  void triangulateNewPoints();

  void removeRedundantKfs();

  inline bool isIdle() const {
    // FIXME Need lock here?
    u_lock lock(mutex_);
    return is_idle_.load();
  }

  void reset();

  // Setters to link components.
  void setSystem(sptr<System> system);
  void setTracker(sptr<Tracking> tracker);
  void setMap(sptr<Map> map);
  void setVocabulary(const sptr<Vocabulary>& voc);
  void setKeyframeDB(KeyframeDB::Ptr keyframe_db);

 private:
  queue<Frame::Ptr> kfs_queue_;  // Keyframes queue waiting to be processed.
  Frame::Ptr curr_keyframe_;     // The keyframe currently under processing.

  // Multi-threading stuff.
  uptr<std::thread> thread_ = nullptr;
  std::condition_variable new_kf_cond_var_;
  std::atomic<bool> is_running_;
  mutable std::mutex mutex_;

  sptr<System> system_ = nullptr;
  sptr<Tracking> tracker_ = nullptr;
  sptr<Map> map_ = nullptr;
  KeyframeDB::Ptr keyframe_db_ = nullptr;
  sptr<Vocabulary> voc_ = nullptr;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_BACK_END_LOCAL_MAPPING_H_