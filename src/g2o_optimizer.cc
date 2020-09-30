#include "mono_slam/g2o_optimizer.h"

#include "mono_slam/common_include.h"
#include "mono_slam/feature.h"
#include "mono_slam/g2o_optimizer/g2o_utils.h"
#include "mono_slam/map_point.h"

namespace mono_slam {

// FIXME Do we need to add a critical section whenever we access a shared
// variable?

void globalBA(const Map::Ptr& map, const int n_iters = 20) {
  // Setup g2o optimizer.
  g2o::SparseOptimizer optimizer;
  g2o_utils::setupG2oOptimizer(&optimizer);

  const list<Frame::Ptr>& kfs = map->GetAllkeyframes();
  // Chi-square test threshold used as the width of the robust huber kernel and
  // rejection of outliers in post-processing.
  const double chi2_thresh = 5.991;
  // Edges container used for post-processing.
  list<uptr<g2o_types::EdgeContainer>> edge_container;

  // Iterate all keyframes in the map.
  int v_id = 0;  // Vertex id.
  for (const Frame::Ptr& kf : kfs) {
    // Create frame vertex. Fixed if it's the first frame.
    kf->v_frame_ = g2o_utils::createG2oVertexFrame(kf, v_id++, kf->id_ == 0);
    assert(optimizer.addVertex(kf->v_frame_));
    // Iterate all features and linked map points observed by this keyframe.
    // FIXME .lock() changes states?
    // FIXME Frequent weak_ptr.lock() operations incur large overhead?
    for (const Feature::Ptr& feat : kf->feats_) {
      const MapPoint::Ptr& point = feat_utils::getPoint(feat);
      if (!point || point->v_point_ == nullptr) {
        // FIXME Does g2o need contiguous ids?
        point->v_point_ = g2o_utils::createG2oVertexPoint(point, v_id++);
      }
      // Low weight of high level features since high image pyramid level
      // (possibly) corrsponds to large error.
      //! "1. / (1 << level)" to account the level 0 case.
      const auto& e_obs = g2o_utils::createG2oEdgeObs(
          kf->v_frame_, point->v_point_, feat->pt_, 1. / (1 << feat->level_),
          std::sqrt(chi2_thresh));
      assert(optimizer.addVertex(point->v_point_));
      optimizer.addEdge(e_obs);
      edge_container.emplace_back(e_obs, kf, feat);
    }
  }

  // Run g2o optimizer.
  g2o_utils::runG2oOptimizer(&optimizer, n_iters);

  // Update structure and motion.
  for (const Frame::Ptr& kf : kfs) {
    kf->setPose(kf->v_frame_->estimate());
    // FIXME reset or set to nullptr?
    kf->v_frame_.reset();
    for (const Feature::Ptr& feat : kf->feats_) {
      const MapPoint::Ptr& point = feat_utils::getPoint(feat);
      if (!point || point->v_point_ == nullptr)
        continue;  // If map point was updated before.
      point->setPos(point->v_point_->estimate());
      point->v_point_.reset();
    }
  }

  // Remove bad observations (i.e. ones incur large reprojection error).
  for (const auto& edge : edge_container) {
    if (edge.e_obs_->chi2() > chi2_thresh) {
      // TODO(bayes) These marking logic can be wrapped into removeObservation()
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

int optimizePose(const Frame::Ptr& frame, const int n_iters = 10) {
  // Setup g2o optimizer.
  g2o::SparseOptimizer optimizer;
  g2o_utils::setupG2oOptimizer(&optimizer);

  // Chi-square test threshold used as the width of the robust huber kernel and
  // for rejection of outliers during post-processing.
  const double chi2_thresh = 5.991;
  // Store the added edges used for post-processing.
  list<uptr<g2o_types::EdgeContainerPoseOnly>> edge_container;

  // Create g2o Frame vertex.
  frame->v_frame_ = g2o_utils::createG2oVertexFrame(frame, frame->id_);
  assert(optimizer.addVertex(frame->v_frame_));

  // Iterate all frame->features->map_points.
  for (const Feature::Ptr& feat : frame->feats_) {
    const MapPoint::Ptr& point = feat_utils::getPoint(feat);
    if (!point) continue;
    // Create g2o pose-only unary edge.
    const auto& e_pose_only = g2o_utils::createG2oEdgePoseOnly(
        frame->v_frame_, feat.lock()->pt_, point->pos_, frame->cam_->K(),
        1. / (1 << feat->level_), std::sqrt(chi2_thresh));
    assert(optimizer.addEdge(e_pose_only));
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
  frame->v_frame_.reset();
  return final_num_inliers;
}

void localBA(const Frame::Ptr& keyframe, const Map::Ptr& map,
             const int n_iters = 5) {
  // Setup g2o optimizer.
  g2o::SparseOptimizer optimizer;
  g2o_utils::setupG2oOptimizer(&optimizer);

  // Obtain covisible keyframes which are then going to be optimized.
  const set<Frame::Ptr>& co_kfs = keyframe->getCovisibleKeyframes();

  // Chi-square test threshold used as the width of the robust huber kernel and
  // for rejection of outliers during post-processing.
  const double chi2_thresh = 5.991;

  //! The reason we choose "list" data structure is that the capacity of the
  //! container is unknown and cannot be predicted precisely in advance which
  //! excludes the most general container -- "vector".
  // Store the added g2o edges and their vertices.
  list<uptr<g2o_types::EdgeContainer>> edge_container;
  // Store the map points to be optimized.
  list<sptr<MapPoint>> points;
  // Store the keyframes involved in optimization while fixed.
  list<sptr<Frame::Ptr>> fixed_kfs;

  // Iterate all covisible keyframes.
  int v_id = 0;  // Vertex id.
  for (const Frame::Ptr& kf : core_kfs) {
    // Fixed if it's the first frame.
    kf->v_frame_ = g2o_utils::createG2oVertexFrame(kf, v_id++, kf->id_ == 0);
    assert(optimizer.addVertex(kf->v_frame_));

    // Iterate all map points observed by this keyframe.
    for (const Feature::Ptr& feat : kf->feats_) {
      const MapPoint::Ptr& point = feat_utils::getPoint(feat);
      if (!point || point->curr_ba_keyframe_id_ == kf->id_) continue;
      point->curr_ba_keyframe_id_ = kf->id_;
      point->v_point_ = g2o_utils::createG2oVertexPoint(point, v_id++);
      assert(optimizer.addVertex(point->v_point_));
      points.push_back(point);
      //! Delay the iteration of observations of each map point to avoid many
      //! repeat comparisons.
      //! Also delay the creation of g2o edges for the same consideration.
    }
  }

  // Iterate all local map points and find their observations.
  for (const MapPoint::Ptr& point : points) {
    for (const Feature::Ptr& feat : point->observations_) {
      const Frame::Ptr& kf = feat_utils::getKeyframe(feat);
      if (!kf) continue;  // If empty shared_ptr or nullptr.
      if (kf->v_frame_ == nullptr) {
        // If does not have a frame yet, kf is selected as afixed keyframe.
        kf->v_frame_ = g2o_utils::createG2oVertexFrame(kf, v_id++, true);
        assert(optimzier.addVertex(kf->v_frame_));
        fixed_kfs.push_back(kf);
      }

      // Creata g2o edge for each valid observation.
      const auto& e_obs = g2o_utils::createG2oEdgeObs(
          kf->v_frame_, point->v_point_, feat->pt_, 1. / (1 << feat->level_),
          std::sqrt(chi2_thresh));
      edge_container.emplace_back(e_obs, kf, feat);
    }
  }

  // Run g2o optimizer.
  g2o_utils::runG2oOptimizer(&optimizer, n_iter);

  // Filter out edges having large reprojection error or negative depth value.
  for (const auto& edge : edge_container) {
    auto& e_obs = edge->e_obs_;
    // FIXME Why only do positive depth checking at here only?
    if (e_obs_->chi2() > chi2_thresh || !e_obs_->isDepthPositive())
      e_obs->setLevel(1);  // Not involved in optimization from now on.
    e_obs->setRobustKernel(nullptr);  // Not using robust kernel from now on.
  }

  // Run g2o optimizer again.
  g2o_utils::runG2oOptimizer(&optimizer, n_iter);

  // Update structure and motion.
  for (const Frame::Ptr& kf : co_kfs) {
    kf->setPose(kf->v_frame_->estimate());
    kf->v_frame_.reset();
  }
  for (const MapPoint::Ptr& point : points) {
    point->setPos(point->v_point_->estimate());
    point->v_point_.reset();
  }
  // Reset fixed keyframes' temporary g2o frame vertex.
  for (const Frame::Ptr& kf : fixed_kfs) kf->v_frame_.reset();

  // Remove observations with too large reprojection error.
  for (const auto& edge : edge_container)
    if (edge->e_obs_->chi2() > chi2_thresh)
      map->removeObservation(edge->keyframe_, edge->feat_);
}

}  // namespace mono_slam