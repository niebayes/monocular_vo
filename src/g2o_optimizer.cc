#include "mono_slam/g2o_optimizer.h"

#include "mono_slam/g2o_optimizer/utils.h"

namespace mono_slam {

void GlobalBundleAdjustment(const Map::Ptr& map,
                            const int num_iterations = 20) {
  // Setup g2o optimizer.
  g2o::SparseOptimizer optimizer;
  g2o_utils::setupG2oOptimizer(&optimizer);

  const list<Frame::Ptr>& keyframes = map->GetAllkeyframes();
  // Chi-square test threshold used as the width of the robust huber kernel and
  // rejection of outliers in post-processing.
  const double chi2_thresh = 5.991;
  // Edges container used for post-processing.
  list<uptr<g2o_types::EdgeContainer>> edge_container;

  // Iterate all keyframes in the map.
  for (const auto& keyframe : keyframes) {
    // Create frame vertex.
    const auto& v_frame =
        g2o_utils::createG2oVertexFrame(keyframe, keyframe->id_);
    // Iterate all features and linked map points observed by this keyframe.
    // FIXME .lock() changes states?
    for (const auto& feat_ : keyframe->feats_) {
      if (feat_.expired()) continue;
      const auto& feat = feat_.lock();
      if (feat->is_outlier_ || feat->point_.expired()) continue;
      const auto& point = feat->point_.lock();
      // FIXME Error with unique_ptr?
      auto& v_point = point->v_point_;
      if (v_point_ == nullptr) {
        v_point = g2o_utils::createG2oVertexPoint(
            point, point->id_ + map->max_frame_id_);
      }
      // Low weight of high level features since high image pyramid level
      // (possibly) corrsponds to large error.
      const auto& e_obs = g2o_utils::createG2oEdgeObs(
          v_frame, v_point, feat, 1. / feat->level_, chi2_thresh);
      edge_container.emplace_back(e_obs, keyframe, feat);

      // Add vertices and edge to optimizer.
      optimizer.addVertex(v_frame);
      optimizer.addVertex(v_point);
      optimizer.addEdge(e_obs);
    }
  }

  // Run g2o optimizer.
  g2o_utils::runG2oOptimizer(&optimizer, num_iterations);

  // Update structure and motion.
  for (const auto& keyframe : keyframes) {
    keyframe->SetPose(keyframe->v_frame_->estimate());
    // FIXME reset or set to nullptr?
    keyframe->v_frame_.reset();
    for (const auto& feat_ : keyframe->feats_) {
      // TODO(bayes) Wrap the logic to a function, e.g. getPointFromFrame().
      if (feat_.expired()) continue;
      const auto& feat = feat_.lock();
      if (feat->is_outlier_ || feat->point_.expired()) continue;
      const auto& point = feat->point_.lock();
      if (point->v_point_ == nullptr)
        continue;  // If map point was updated before.
      point->SetPos(point->v_point_->estimate());
      point->v_point_.reset();
    }
  }

  // Remove bad observations (i.e. ones incur large reprojection error).
  for (const auto& edge : edge_container) {
    if (edge.e_obs_->chi2() > chi2_thresh) {
      edge.feat_->is_outlier_ =
          true  // FIXME Does this help in deletion?
          // FIXME Will it help if map point can be marked outlier?
          edge.feat_->point_.lock()
              ->is_outlier_ = true;
      // TODO(bayes) Wrap the removal logic into the function below.
      map->removeObservation(edge.keyframe_, edge.feat_);
    } else {
      // TODO(bayes) Rewrite. Outlier feature will be deleted in the function
      // above.
      edge.feat_->is_outlier_ = false;
    }
  }
}

}  // namespace mono_slam