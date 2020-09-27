#include "mono_slam/front_end_tracking.h"

namespace mono_slam {

Tracking::Tracking() : state_(Tracking::State::NOT_INITIALIZED_YET) {}

void Tracking::AddImage(const cv::Mat& img) {
  curr_frame_ = make_shared<Frame>(img, cam_, voc_, detector_);
  TrackCurrentFrame();
  // Update last frame.
  last_frame_ = curr_frame_;
  // FIXME Need resetting? What will reset() do on smart point?
  curr_frame_.reset();
}

void Tracking::TrackCurrentFrame() {
  switch (state_) {
    case Tracking::State::NOT_INITIALIZED_YET:
      if (InitMap()) {
        last_keyframe_id_ = curr_frame_->id_;
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
    initializer_->AddCurrentFrame(curr_frame_);
  return initializer_->stage_ == Initializer::Stage::SUCCESS;
}

bool Tracking::TrackWithConstantVelocityModel() {
  //
}

void Tracking::TrackLocalMap() {
  // TODO(bayes) Implement this.
}

bool Tracking::NeedNewKeyframe() {
  //
}

bool Tracking::Relocalization() {
  //
}

void Tracking::SetSystem(sptr<System> system) { system_ = system; }
void Tracking::SetLocalMapper(sptr<LocalMapping> local_mapper) {
  local_mapper_ = local_mapper;
}
void Tracking::SetMap(Map::Ptr map) { map_ = map; }
void Tracking::SetKeyframeDB(KeyframeDB::Ptr keyframe_db) {
  keyframe_db_ = keyframe_db;
}
void Tracking::SetViewer(sptr<Viewer> viewer) { viewer_ = viewer; }
void Tracking::SetInitializer(Initializer::Ptr initializer) {
  initializer_ = initializer;
}
void Tracking::SetVocabulary(const sptr<Vocabulary>& voc) { voc_ = voc; }
void Tracking::SetCamera(const Camera::Ptr& cam) { cam_ = cam; }
void Tracking::SetFeatureDetector(
    const cv::Ptr<cv::FeatureDetector>& feat_detector) {
  feat_detector_ = feat_detector;
}

}  // namespace mono_slam
