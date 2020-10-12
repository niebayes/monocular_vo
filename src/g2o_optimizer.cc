#include "mono_slam/g2o_optimizer.h"

#include "mono_slam/common_include.h"
#include "mono_slam/feature.h"
#include "mono_slam/frame.h"
#include "mono_slam/g2o_optimizer/g2o_types.h"
#include "mono_slam/g2o_optimizer/g2o_utils.h"
#include "mono_slam/map.h"
#include "mono_slam/map_point.h"

// FIXME Possible issues may be raised from the codes calling for obtaining
// references of some objects. Maybe we could trace them to see whether this
// will happen or not.

namespace mono_slam {

struct Feature;
class Frame;
class MapPoint;
class Map;
class Camera;

void Optimizer::globalBA(const Map::Ptr& map, const int n_iters) {
  LOG(INFO) << "Start globalBA: n_iters = " << n_iters;
  const steady_clock::time_point t1 = steady_clock::now();

  // Setup g2o optimizer.
  g2o::SparseOptimizer optimizer;
  g2o_utils::setupG2oOptimizer(&optimizer);

  // Chi-square test threshold used as the width of the robust huber kernel and
  // for rejecting outliers in post-processing.
  const double chi2_thresh = 5.991;  // Two-degree chi-square p-value.
  // Edge container used for post-processing.
  list<g2o_types::EdgeContainer> edge_container;
  // Get keyframes whose poses are going to be optimized.
  const list<Frame::Ptr>& kfs = map->getAllKeyframes();

  // Iterate all keyframes in the map.
  int v_id = 0;  // Vertex id.
  for (const Frame::Ptr& kf : kfs) {
    // Create frame vertex. Fixed if it's the datum frame.
    kf->v_frame_ = g2o_utils::createG2oVertexFrame(kf, v_id++, kf->is_datum_);
    assert(optimizer.addVertex(kf->v_frame_));  // Asserting for debugging.
    // Iterate all features and linked map points observed by this keyframe.
    for (const Feature::Ptr& feat : kf->feats_) {
      const MapPoint::Ptr& point = feat_utils::getPoint(feat);
      if (!point) continue;
      // Avoid repeat vertex creation since there's visual overlapping among
      // frames.
      if (point->v_point_ == nullptr) {
        point->v_point_ = g2o_utils::createG2oVertexPoint(point, v_id++);
        assert(optimizer.addVertex(point->v_point_));
      }
      // Lower weight for high level features since high image pyramid level
      // generally produces larger error.
      //! "1. / (1 << level)" to account for the level 0 case.
      auto e_obs = g2o_utils::createG2oEdgeObs(
          kf->v_frame_, point->v_point_, feat->pt_, kf->cam_->K(),
          1. / (1 << feat->level_), std::sqrt(chi2_thresh));
      assert(optimizer.addEdge(e_obs));
      edge_container.emplace_back(e_obs, kf, feat);
    }
  }

  // Run g2o optimizer.
  double init_error, final_error;
  g2o_utils::runG2oOptimizer(&optimizer, n_iters, init_error, final_error);
  LOG(INFO) << cv::format("globalBA: (init_error: %.4f, final_error: %.4f).",
                          init_error, final_error);

  // Update structure and motion.
  for (const Frame::Ptr& kf : kfs) {
    const SE3 estimate_(kf->v_frame_->estimate().rotation(),
                        kf->v_frame_->estimate().translation());
    kf->setPose(estimate_);
    // Reseat pointer making it ready for next optimization.
    kf->v_frame_ = nullptr;
    for (const Feature::Ptr& feat : kf->feats_) {
      const MapPoint::Ptr& point = feat_utils::getPoint(feat);
      if (!point || point->v_point_ == nullptr)
        continue;  // If map point was updated before.
      point->setPos(point->v_point_->estimate());
      point->v_point_ = nullptr;
    }
  }

  // Remove bad observations (i.e. ones incur large reprojection error).
  for (auto& edge : edge_container) {
    if (edge.e_obs_->chi2() <= chi2_thresh) continue;
    map->removeBadObservations(edge.keyframe_, edge.feat_);
  }

  const steady_clock::time_point t2 = steady_clock::now();
  const double time_span = duration_cast<duration<double>>(t2 - t1).count();
  LOG(INFO) << "globalBA finished in " << time_span << " seconds.";
}

int Optimizer::optimizePose(const Frame::Ptr& frame, const int n_iters) {
  CHECK_EQ(frame->is_datum_, false);  // Cannot be the datum frame.
  LOG(INFO) << "Start optimizePose: n_iters = " << n_iters
            << " each for 4 optimizations.";
  const steady_clock::time_point t1 = steady_clock::now();

  // Setup g2o optimizer.
  g2o::SparseOptimizer optimizer;
  g2o_utils::setupG2oOptimizer(&optimizer);

  // Chi-square test threshold used as the width of the robust huber kernel and
  // for rejecting outliers during post-processing.
  const double chi2_thresh = 5.991;  // Two-degree chi-square p-value.
  // Store the added edges used for post-processing.
  list<g2o_types::EdgeContainerPoseOnly> edge_container;

  // Create g2o Frame vertex.
  frame->v_frame_ = g2o_utils::createG2oVertexFrame(frame, frame->id_);
  assert(optimizer.addVertex(frame->v_frame_));

  // Iterate all frame->features->map_points.
  for (const Feature::Ptr& feat : frame->feats_) {
    const MapPoint::Ptr& point = feat_utils::getPoint(feat);
    if (!point) continue;
    // Create g2o pose-only unary edge.
    auto e_pose_only = g2o_utils::createG2oEdgePoseOnly(
        frame->v_frame_, feat->pt_, point->pos_, frame->cam_->K(),
        1. / (1 << feat->level_), std::sqrt(chi2_thresh));
    assert(optimizer.addEdge(e_pose_only));
    edge_container.emplace_back(e_pose_only, feat);
  }

  // Alternatively perform 4 optimizations each for 10 iterations. Classify
  // inliers / outliers at each optimization with the inliers only passed into
  // the next optimization whilst the outliers are classified again in the next
  // optimization.
  int final_num_inliers = 0;  // Number of inliers to be returned.
  double init_error, final_error;
  const g2o::SE3Quat init_pose(frame->pose().rotationMatrix(),
                               frame->pose().translation());
  for (int i = 0; i < 4; ++i) {
    // Reset initial pose estimate in case that the last optimization
    // makes it diverged.
    frame->v_frame_->setEstimate(init_pose);
    // Run g2o optimizer.
    g2o_utils::runG2oOptimizer(&optimizer, n_iters, init_error, final_error);
    LOG(INFO) << cv::format(
        "optimizePose(%d): (init_error: %.4f, final_error: %.4f).", i + 1,
        init_error, final_error);

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
        ++final_num_inliers;
      }

      // Only use robust kernel in the first two optimizations since
      // robustification export slight overhead.
      if (i == 1) edge.e_pose_only_->setRobustKernel(nullptr);
    }
  }

  // Update frame pose.
  const SE3 estimate_(frame->v_frame_->estimate().rotation(),
                      frame->v_frame_->estimate().translation());
  frame->setPose(estimate_);
  frame->v_frame_ = nullptr;

  //! We delay the removal of bad observations since current information may not
  //! be sufficient to completely judge the goodness of an observation.

  const steady_clock::time_point t2 = steady_clock::now();
  const double time_span = duration_cast<duration<double>>(t2 - t1).count();
  LOG(INFO) << "optimizePose finished in " << time_span << " seconds.";
  return final_num_inliers;
}

void Optimizer::localBA(const Frame::Ptr& keyframe, const Map::Ptr& map,
                        const int n_iters) {
  LOG(INFO) << "Start localBA: n_iters = " << n_iters
            << " each for 2 optimizations.";
  const steady_clock::time_point t1 = steady_clock::now();

  // Setup g2o optimizer.
  g2o::SparseOptimizer optimizer;
  g2o_utils::setupG2oOptimizer(&optimizer);

  //! The reason we choose "list" data structure is that the capacity of the
  //! container is unknown and cannot be predicted precisely in advance which
  //! excludes the most general container -- "vector".

  // Chi-square test threshold used as the width of the robust huber kernel and
  // for rejecting outliers during post-processing.
  const double chi2_thresh = 5.991;  // Two-degree chi-square p-value.
  // Store the added g2o edges and their vertices for post-processing.
  list<g2o_types::EdgeContainer> edge_container;
  // Store the map points to be optimized.
  list<MapPoint::Ptr> points;
  // Store the keyframes involved in optimization while fixed.
  list<Frame::Ptr> fixed_kfs;
  // Obtain covisible keyframes which are then going to be optimized.
  const forward_list<Frame::Ptr>& co_kfs = keyframe->getCoKfs();

  // Iterate all covisible keyframes.
  //! The covisible information was updated before.
  int v_id = 0;  // Vertex id.
  for (const Frame::Ptr& kf : co_kfs) {
    // Fixed if it's the datum frame.
    //! The datum frame may not ever be passed into here. (Or never?)
    kf->v_frame_ = g2o_utils::createG2oVertexFrame(kf, v_id++, kf->is_datum_);
    assert(optimizer.addVertex(kf->v_frame_));

    // Iterate all map points observed by this keyframe.
    for (const Feature::Ptr& feat : kf->feats_) {
      const MapPoint::Ptr& point = feat_utils::getPoint(feat);
      // Avoid repeat point vertex creation.
      if (!point || point->curr_ba_keyframe_id_ == kf->id_) continue;
      if (point->curr_ba_keyframe_id_ == kf->id_) continue;
      point->curr_ba_keyframe_id_ = kf->id_;
      point->v_point_ = g2o_utils::createG2oVertexPoint(point, v_id++);
      assert(optimizer.addVertex(point->v_point_));
      points.push_back(point);
      //! Delay the iteration of observations of each map point to avoid many
      //! repeat comparisons.
      //! Also delay the creation of g2o edges for the same consideration.
    }
  }

  // Iterate all local map points and obtain their observations.
  for (const MapPoint::Ptr& point : points) {
    for (const Feature::Ptr& feat : point->getObservations()) {
      const Frame::Ptr& kf = feat_utils::getKeyframe(feat);
      if (!kf) continue;  // FIXME This should never happen.
      if (kf->v_frame_ == nullptr) {
        // If does not have a frame yet, kf is selected as a fixed keyframe.
        kf->v_frame_ = g2o_utils::createG2oVertexFrame(kf, v_id++, true);
        assert(optimizer.addVertex(kf->v_frame_));
        fixed_kfs.push_back(kf);
      }

      // Creata g2o edge for each valid observation.
      auto e_obs = g2o_utils::createG2oEdgeObs(
          kf->v_frame_, point->v_point_, feat->pt_, kf->cam_->K(),
          1. / (1 << feat->level_), std::sqrt(chi2_thresh));
      assert(optimizer.addEdge(e_obs));
      edge_container.emplace_back(e_obs, kf, feat);
    }
  }

  //! Two separate optimizations with the first to exclude outliers while the
  //! second to solid the estimate.

  // Run g2o optimizer.
  double init_error, final_error;
  g2o_utils::runG2oOptimizer(&optimizer, n_iters, init_error, final_error);
  LOG(INFO) << cv::format("localBA(1): (init_error: %.4f, final_error: %.4f).",
                          init_error, final_error);

  // Filter out edges having large reprojection error.
  for (const auto& edge : edge_container) {
    auto& e_obs = edge.e_obs_;
    if (e_obs->chi2() > chi2_thresh)
      e_obs->setLevel(1);  // Not involved in optimization from now on.
    e_obs->setRobustKernel(nullptr);  // Not using robust kernel from now on.
  }

  // Run g2o optimizer again.
  g2o_utils::runG2oOptimizer(&optimizer, n_iters, init_error, final_error);
  LOG(INFO) << cv::format("localBA(2): (init_error: %.4f, final_error: %.4f).",
                          init_error, final_error);

  // Remove bad observations with too large reprojection error.
  for (auto& edge : edge_container) {
    if (edge.e_obs_->chi2() <= chi2_thresh) continue;
    map->removeBadObservations(edge.keyframe_, edge.feat_);
  }

  // Update structure and motion.
  for (const Frame::Ptr& kf : co_kfs) {
    const SE3 estimate_(kf->v_frame_->estimate().rotation(),
                        kf->v_frame_->estimate().translation());
    kf->setPose(estimate_);
    kf->v_frame_ = nullptr;
  }
  for (const MapPoint::Ptr& point : points) {
    // FIXME Have no idea why this would happen.
    if (point->v_point_ == nullptr) continue;
    point->setPos(point->v_point_->estimate());
    point->v_point_ = nullptr;
  }
  // Reset fixed keyframes' temporary g2o frame vertices.
  for (const Frame::Ptr& kf : fixed_kfs) kf->v_frame_ = nullptr;

  const steady_clock::time_point t2 = steady_clock::now();
  const double time_span = duration_cast<duration<double>>(t2 - t1).count();
  LOG(INFO) << "localBA finished in " << time_span << " seconds.";
}

}  // namespace mono_slam