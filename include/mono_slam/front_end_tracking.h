#ifndef MONO_SLAM_FRONT_END_TRACKING_H_
#define MONO_SLAM_FRONT_END_TRACKING_H_

#include "mono_slam/back_end_local_mapping.h"
#include "mono_slam/common_include.h"
#include "mono_slam/frame.h"
#include "mono_slam/front_end_map_initialization.h"
#include "mono_slam/keyframe_database.h"
#include "mono_slam/map.h"
#include "mono_slam/system.h"
#include "mono_slam/viewer.h"

namespace mono_slam {

class System;
class LocalMapping;
class Map;
class Frame;
class Viewer;
class Initializer;
class KeyFrameDB;

class Tracking {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = sptr<Tracking>;

  enum class State { NOT_INITIALIZED_YET, GOOD, LOST };

  Tracking() {}

  bool TrackOneFrame(Frame::Ptr frame);

  // Setters.
  void SetSystem(sptr<System> system);
  void SetLocalMapper(sptr<LocalMapping> local_mapper);
  void SetMap(Map::Ptr map);
  void SetViewer(sptr<Viewer> viewer);
  void SetInitializer(Initializer::Ptr initializer);
  void SetVocabulary(const sptr<Vocabulary>& voc);
  void SetKeyframeDB(KeyframeDB::Ptr keyframe_db);

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
  // Linked components.
  sptr<System> system_ = nullptr;              // System.
  sptr<LocalMapping> local_mapper_ = nullptr;  // Local mapper.
  Map::Ptr map_ = nullptr;                     // Map.
  sptr<Viewer> viewer_ = nullptr;              // Viewer.
  Initializer::Ptr initializer_ = nullptr;      // Initializer.
  sptr<Vocabulary> voc_ = nullptr;
  KeyframeDB::Ptr keyframe_db_ = nullptr;  // Keyframe database.
};

bool Tracking::TrackOneFrame(Frame::Ptr frame) {
  //
}

void Tracking::SetSystem(sptr<System> system) { system_ = system; }
void Tracking::SetLocalMapper(sptr<LocalMapping> local_mapper) {
  local_mapper_ = local_mapper;
}
void Tracking::SetMap(Map::Ptr map) { map_ = map; }
void Tracking::SetViewer(sptr<Viewer> viewer) { viewer_ = viewer; }
void Tracking::SetInitializer(Initializer::Ptr initializer) {
  initializer_ = initializer;
}
void Tracking::SetVocabulary(const sptr<Vocabulary>& voc) { voc_ = voc; }

}  // namespace mono_slam

#endif  // MY_FRONT_END_TRACKING