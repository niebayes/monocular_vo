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

  Tracking();

  // Entry function.
  void AddImage(const cv::Mat& img);

  // Setters.
  void SetSystem(sptr<System> system);
  void SetLocalMapper(sptr<LocalMapping> local_mapper);
  void SetMap(Map::Ptr map);
  void SetKeyframeDB(KeyframeDB::Ptr keyframe_db);
  void SetViewer(sptr<Viewer> viewer);
  void SetInitializer(Initializer::Ptr initializer);
  void SetVocabulary(const sptr<Vocabulary>& voc);
  void SetCamera(const Camera::Ptr& cam);
  void SetFeatureDetector(const cv::Ptr<FeatureDetector>& detector);

  void Reset();

 private:
  // Track current frame.
  void TrackCurrentFrame();

  // Initialize map: collect two consecutive frames and try initialization.
  bool InitMap();

  // Track current frame from last frame assuming contant velocity model.
  bool TrackWithConstantVelocityModel();

  // True if the criteria of inserting new keyframe are satisfied.
  bool NeedNewKeyframe();

  // Relocalize if tracking is lost.
  bool Relocalization();

 public:
  // Tracking state.
  Tracking::State state_;

  // Tracker only maintains two frames at one moment.
  Frame::Ptr last_frame_ = nullptr;
  Frame::Ptr curr_frame_ = nullptr;
  // Matches of last frame and current frame such that
  // last_frame_[i] <-> curr_frame_[matches[i]].
  vector<int> matches_;

  // The transformation from last frame to current frame assuming constant
  // velocity.
  SE3 const_velocity_;

 private:
  // Linked components.
  sptr<System> system_ = nullptr;              // System.
  sptr<LocalMapping> local_mapper_ = nullptr;  // Local mapper.
  Map::Ptr map_ = nullptr;                     // Map.
  KeyframeDB::Ptr keyframe_db_ = nullptr;      // Keyframe database.
  sptr<Viewer> viewer_ = nullptr;              // Viewer.

  // User specified objects.
  sptr<Initializer> initializer_ = nullptr;           // Initializer.
  sptr<Vocabulary> voc_ = nullptr;                   // Vocabulary.
  Camera::Ptr cam_ = nullptr;                        // Camera.
  cv::Ptr<cv::FeatureDetector> detector_ = nullptr;  // Feature detector.
};

Tracking::Tracking() : state_(Tracking::State::NOT_INITIALIZED_YET) {}

void Tracking::AddImage(const cv::Mat& img) {
  curr_frame_ = make_shared<Frame>(img, cam_, voc_, detector_);
  TrackCurrentFrame();
  // Update last frame.
  last_frame_ = curr_frame_;
  // FIXME Need resetting?
  curr_frame_.reset();
}

void Tracking::TrackCurrentFrame() {
  switch (state_) {
    case Tracking::State::NOT_INITIALIZED_YET:
      if (InitMap()) {
        last_frame_->SetKeyframe();
        curr_frame_->SetKeyframe();
        local_mapper_->InsertKeyframe(last_frame_);
        local_mapper_->InsertKeyframe(curr_frame_);
        state_ = Tracking::State::GOOD;
      }
      break;

    case Tracking::State::GOOD:
      if (TrackWithConstantVelocityModel()) {
        TrackLocalMap();  // Track local map making the tracking more robust.
        if (NeedNewKeyframe()) {
          curr_frame_->SetKeyframe();
          local_mapper_->InsertKeyframe(curr_frame_);
        }
      } else
        state_ = Tracking::State::LOST;
      break;

    case Tracking::State::LOST:
      if (Relocalization()) {
        curr_frame_->SetKeyframe();
        local_mapper_->InsertKeyframe(curr_frame_);
        state_ = Tracking::State::GOOD;
      } else
        Reset();
      break;
  }
}

bool Tracking::InitMap() {
  if (initializer_->stage_ == Initializer::Stage::NO_FRAME_YET)
    initializer_->AddReferenceFrame(curr_frame_);
  else
    initializer_->AddCurrentFrame(curr_frame_, matches_);
  return initializer_->stage_ == Initializer::Stage::SUCCESS;
}

bool Tracking::TrackWithConstantVelocityModel() {
  //
}

bool Tracking::NeedNewKeyframe() {
  //
}

bool Tracking::Relocalization() {
  //
}
namespace tracking_utils {}  // namespace tracking_utils

}  // namespace mono_slam

#endif  // MY_FRONT_END_TRACKING