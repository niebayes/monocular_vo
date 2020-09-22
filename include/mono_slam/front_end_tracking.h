#ifndef MONO_SLAM_FRONT_END_TRACKING_H_
#define MONO_SLAM_FRONT_END_TRACKING_H_

#include "mono_slam/back_end_local_mapping.h"
#include "mono_slam/common_include.h"
#include "mono_slam/feature_manager.h"
#include "mono_slam/frame.h"
#include "mono_slam/front_end_map_initialization.h"
#include "mono_slam/map.h"
#include "mono_slam/slam_system.h"
#include "mono_slam/viewer.h"

namespace mono_slam {

class System;
class LocalMapping;
class Map;
class Frame;
class FeatureManager;
class Viewer;
class Initializer;

class Tracking {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = sptr<Tracking>;

  enum class State {
    NOT_INITIALIZED_YET,
    GOOD,
    LOST
  };

  Tracking();

  void SetLocalMapper(sptr<LocalMapping> local_mapper);

  void SetMap(Map::Ptr map);

  void SetViewer(sptr<Viewer> viewer);

  void Reset();

 private:
  // Extract features.
  void ExtractFeatures(const cv::Mat& img);

 public:
  // Tracking state.
  Tracking::State state_;

  // Tracker only maintains two frames at one moment.
  Frame::Ptr last_frame_ = nullptr;
  Frame::Ptr curr_frame_ = nullptr;

  // The transformation from last frame to current frame assuming constant
  // velocity.
  SE3 const_velocity_;

 private:
  // Initializer.
  Initializer::Ptr initialzer_ = nullptr;

  // Handles.
  sptr<System> system_ = nullptr;              // System.
  sptr<LocalMapping> local_mapper_ = nullptr;  // Local mapper.
  Map::Ptr map_ = nullptr;                     // Map.
  sptr<Viewer> viewer_ = nullptr;              // Viewer.
};

}  // namespace mono_slam

#endif  // MY_FRONT_END_TRACKING