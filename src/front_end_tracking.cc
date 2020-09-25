#include "mono_slam/front_end_tracking.h"

namespace mono_slam {

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
