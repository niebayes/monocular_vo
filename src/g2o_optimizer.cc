#include "mono_slam/g2o_optimizer.h"

#include "mono_slam/common_include.h"
#include "mono_slam/feature.h"
#include "mono_slam/g2o_optimizer/g2o_utils.h"
#include "mono_slam/map_point.h"

namespace mono_slam {

void globalBA(const Map::Ptr& map, const int n_iters = 20) {
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
    keyframe->v_frame_ =
        g2o_utils::createG2oVertexFrame(keyframe, keyframe->id_);
    // Iterate all features and linked map points observed by this keyframe.
    // FIXME .lock() changes states?
    for (const auto& feat : keyframe->feats_) {
      const auto& point = feat_utils::getPoint(feat.lock());
      if (!point || point->v_point_ == nullptr) {
        // FIXME Does g2o need contiguous ids?
        point->v_point_ = g2o_utils::createG2oVertexPoint(
            point, point->id_ + map->max_frame_id_);
      }
      // Low weight of high level features since high image pyramid level
      // (possibly) corrsponds to large error.
      const auto& e_obs = g2o_utils::createG2oEdgeObs(
          keyframe->v_frame_, point->v_point_, feat.lock()->pt_,
          1. / feat.lock()->level_, chi2_thresh);
      edge_container.emplace_back(e_obs, keyframe, feat.lock());

      // Add vertices and edge to optimizer.
      optimizer.addVertex(keyframe->v_frame_);
      optimizer.addVertex(point->v_point_);
      optimizer.addEdge(e_obs);
    }
  }

  // Run g2o optimizer.
  g2o_utils::runG2oOptimizer(&optimizer, n_iters);

  // Update structure and motion.
  for (const auto& keyframe : keyframes) {
    keyframe->setPose(keyframe->v_frame_->estimate());
    // FIXME reset or set to nullptr?
    keyframe->v_frame_.reset();
    for (const auto& feat_ : keyframe->feats_) {
      const auto& point = feat_utils::getPoint(feat_.lock());
      if (!point || point->v_point_ == nullptr)
        continue;  // If map point was updated before.
      point->setPos(point->v_point_->estimate());
      point->v_point_.reset();
    }
  }

  // Remove bad observations (i.e. ones incur large reprojection error).
  for (const auto& edge : edge_container) {
    if (edge.e_obs_->chi2() > chi2_thresh) {
      // FIXME Will it help if a feature can be marked as outlier?
      edge.feat_->is_outlier_ = true;  // FIXME Does this help in deletion?
      // FIXME Will it help if map point can be marked as outlier?
      edge.feat_->point_.lock()->is_outlier_ = true;
      // TODO(bayes) Wrap the removal logic into the function below.
      map->removeObservation(edge.keyframe_, edge.feat_);
    } else {
      // TODO(bayes) Rewrite. Outlier feature will be deleted in the function
      // above.
      edge.feat_->is_outlier_ = false;
    }
  }
}

void optimizePose(const sptr<Frame>& frame, const int n_iters = 10) {
  // Setup g2o optimizer.
  g2o::SparseOptimizer optimizer;
  g2o_utils::setupG2oOptimizer(&optimizer);

  // Chi-square test threshold used as the width of the robust huber kernel and
  // for rejection of outliers during post-processing.
  const double chi2_thresh = 5.991;
  list<uptr<g2o_types::EdgeContainerPoseOnly>> edge_container;

  // Create g2o Frame vertex.
  frame->v_frame_ = g2o_utils::createG2oVertexFrame(frame, frame->id_);

  // Iterate all frame->features->map_points.
  for (const auto& feat : frame->feats_) {
    const auto& point = feat_utils::getPoint(feat.lock());
    if (!point) continue;
    // Create g2o pose-only unary edge.
    const auto& e_pose_only = g2o_utils::createG2oEdgePoseOnly(
        frame->v_frame_, feat.lock()->pt_, point->pos_, frame->cam_->K(),
        1. / feat.lock()->level_, chi2_thresh);
    edge_container.emplace_back(e_pose_only, feat.lock());
  }

  // Alternatively perform 4 optimizations each for 10 iterations. Classify
  // inliers / outliers at each optimization with the inliers only passed into
  // the next optimization whilst the outliers are classified again in the next
  // optimization.
  int final_num_inliers = 0;  // Number of inliers to be returned.
  for (int i = 0; i < 4; ++i) {
    // Reset initial pose estimate in case that the last optimization make it
    // diverged.
    frame->v_frame_->setEstimate(frame->pose());
    // Run g2o optimizer.
    g2o_utils::runG2oOptimizer(&optimizer, n_iters);

    final_num_inliers = 0;  // Reset at each optimization.
    for (const auto& edge : edge_container) {
      if (edge.feat_->is_outlier_) {
        // Compute residual at this moment since it's not computed due to
        // classified as outlier in the last optimization
        edge.e_pose_only_->computeError();
      }

      // Classify inliers / outliers.
      if (edge.e_pose_only_->chi2() > chi2_thresh) {
        edge.feat_->is_outlier_ = true;
        edge.e_pose_only_->setLevel(1);  // Not considered in next optimization.
      } else {
        edge.feat_->is_outlier_ = false;
        edge.e_pose_only_->setLevel(0);
      }

      // Only use robust kernel in the first two optimizations.
      // FIXME Robustifisation incurs large overhead?
      if (i == 2) edge.e_pose_only_->setRobustKernel(nullptr);
    }
  }

  // TODO(bayes) Remove measurements with too large errors.
  // FIXME Shall we do this right now?

  // Update frame pose.
  frame->setPose(frame->v_frame_->estimate());
  return final_num_inliers;
}

void localBA() {}

}  // namespace mono_slam