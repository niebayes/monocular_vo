#ifndef MONO_SLAM_BACK_END_LOCAL_MAPPING_H_
#define MONO_SLAM_BACK_END_LOCAL_MAPPING_H_

#include "mono_slam/common_include.h"
#include "mono_slam/frame.h"
#include "mono_slam/map.h"
#include "mono_slam/system.h"
#include "mono_slam/tracking.h"

namespace mono_slam {

class System;
class Tracking;
class Map;
class Frame;

class LocalMapping {
 public:
  LocalMapping();

  void startThread();

  void stopThread();

  void insertKeyframe(Frame::Ptr keyframe);

  void LocalMappingLoop();

  void processFrontKeyframe();

  void triangulateNewPoints();

  void removeRedundantKfs();

  inline bool isIdle() const {
    u_lock lock(mutex_);
    return is_idle_;
  }

  void reset();

  // Setters to link components.
  void setSystem(sptr<System> system);
  void setTracker(sptr<Tracking> tracker);
  void setMap(Map::Ptr map);

 protected:
  queue<Frame::Ptr> kfs_queue_;  // Keyframes queue waiting to be processed.
  Frame::Ptr curr_keyframe_;     // The keyframe currently under processing.

  // Multi-threading stuff.
  std::thread thread_;
  std::condition_variable new_kf_cond_var_;
  std::atomic<bool> is_running_;
  bool is_idle_;
  mutable std::mutex mutex_;

  sptr<System> system_ = nullptr;
  sptr<Tracking> tracker_ = nullptr;
  Map::Ptr map_ = nullptr;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_BACK_END_LOCAL_MAPPING_H_