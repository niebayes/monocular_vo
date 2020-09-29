#include "mono_slam/front_end_tracking.h"

namespace mono_slam {

Tracking::Tracking() : state_(State::NOT_INITIALIZED_YET) {}

void Tracking::addImage(const cv::Mat& img) {
  // FIXME Would it be better if using raw pointer for camera?
  curr_frame_ = make_shared<Frame>(img, std::move(cam_), voc_, detector_);
  trackCurrentFrame();
  // Update constant velocity model, aka. relative motion.
  T_curr_last_ = curr_frame_->pose() * last_frame_->pose().inverse();
  last_frame_ = curr_frame_;  // Update last frame.
  curr_frame_.reset();        // Reseat to make it ready for next frame.
}

void Tracking::trackCurrentFrame() {
  switch (state_) {
    case Tracking::State::NOT_INITIALIZED_YET:
      if (initMap()) {
        last_keyframe_id_ = curr_frame_->id_;
        local_mapper_->insertKeyframe(last_frame_);
        local_mapper_->insertKeyframe(curr_frame_);
        state_ = Tracking::State::GOOD;
      }
      break;

    case Tracking::State::GOOD:
      if (!trackFromLastFrame() || !trackFromLocalMap())
        state_ = State::LOST;
      else {
        if (needNewKeyframe()) {
          curr_frame_->setKeyframe();
          local_mapper_->insertKeyframe(curr_frame_);
        }
      }
      break;

    case Tracking::State::LOST:
      if (relocalization()) {
        curr_frame_->setKeyframe();
        local_mapper_->insertKeyframe(curr_frame_);
        state_ = Tracking::State::GOOD;
      } else
        reset();
      break;
  }
}

bool Tracking::initMap() {
  if (initializer_->stage() == Initializer::Stage::NO_FRAME_YET)
    initializer_->addReferenceFrame(curr_frame_);
  else
    initializer_->addCurrentFrame(curr_frame_);
  return initializer_->stage() == Initializer::Stage::SUCCESS;
}

bool Tracking::trackFromLastFrame() {
  // Set initial pose.
  curr_frame_->setPose(T_curr_last_ * last_frame_->pose());
  // Search matches by projection (i.e. project map points observed by last
  // frame onto current frame and try matching them against features around
  // them).
  const int num_matches = Matcher::searchByProjection(last_frame_, curr_frame_);
  if (num_matches < Config::min_num_matches()) return false;
  const int num_inlier_matches = Optimizer::optimizePose(curr_frame_);
  if (num_inlier_matches < Config::min_num_inlier_matches()) return false;
  return true;
}

bool Tracking::trackFromLocalMap() {
  updateLocalCovisibleKeyframes();
  const int num_matches =
      Matcher::searchByProjection(local_co_kfs_, curr_frame_);
  if (num_matches < Config::min_num_matches()) return false;
  const int num_inlier_matches = Optimizer::optimizePose(curr_frame_);
  if (num_inlier_matches < Config::min_num_inlier_matches()) return false;
  return true;
}

void updateLocalCovisibleKeyframes() {
  local_co_kfs_.clear();
  //
}

bool Tracking::needNewKeyframe() {
  //
  return true;
}

bool Tracking::relocalization() {
  //
  return true;
}

void reset() {
  state_ = State::NOT_INITIALIZED_YET;
  last_frame_.reset();
  curr_frame_.reset();
  T_curr_last_.setZero();
  local_co_kfs_.clear();
  last_keyframe_id_ = 0;
}

void Tracking::setSystem(sptr<System> system) { system_ = system; }
void Tracking::setLocalMapper(sptr<LocalMapping> local_mapper) {
  local_mapper_ = local_mapper;
}
void Tracking::setMap(Map::Ptr map) { map_ = map; }
void Tracking::setKeyframeDB(KeyframeDB::Ptr keyframe_db) {
  keyframe_db_ = keyframe_db;
}
void Tracking::setViewer(sptr<Viewer> viewer) { viewer_ = viewer; }
void Tracking::setInitializer(uptr<Initializer> initializer) {
  initializer_ = std::move(initializer);
}
void Tracking::setVocabulary(const sptr<Vocabulary>& voc) { voc_ = voc; }
void Tracking::setCamera(Camera::Ptr cam) { cam_ = std::move(cam); }
void Tracking::setFeatureDetector(
    const cv::Ptr<cv::FeatureDetector>& detector) {
  detector_ = detector;
}

}  // namespace mono_slam
