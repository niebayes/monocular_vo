#ifndef MONO_SLAM_G2O_OPTIMIZER_UTILS_H_
#define MONO_SLAM_G2O_OPTIMIZER_UTILS_H_

#include "mono_slam/camera.h"
#include "mono_slam/common_include.h"
#include "mono_slam/frame.h"
#include "mono_slam/g2o_optimizer/g2o_types.h"

namespace mono_slam {
namespace g2o_utils {

void setupG2oOptimizer(g2o::SparseOptimizer* optimizer, const Mat33& K) {
  // Set solver.
  //! Even though we "new" a lot of things without delete, g2o
  //! internally takes care of them implicitly. Hence no memory leak.
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      g2o::make_unique<g2o_types::BlockSolver>(
          g2o::make_unique<g2o_types::LinearSolver>()));
  optimizer->setAlgorithm(solver);
}

void runG2oOptimizer(g2o::SparseOptimizer* optimizer, const int n_iters,
                            double& init_error, double& final_error) {
  // FIXME Any useful options?
  optimizer->initializeOptimization();
  optimizer->setVerbose(true);
  optimizer->computeActiveErrors();
  init_error = optimizer->activeChi2();
  optimizer->optimize(n_iters);
  final_error = optimizer->activeChi2();
}

g2o_types::VertexFrame* createG2oVertexFrame(
    const Frame::Ptr& keyframe, const int id, const bool is_fixed = false) {
  auto v_frame = new g2o_types::VertexFrame();
  const SE3& pose = keyframe->pose();
  v_frame->setEstimate(
      g2o::SE3Quat(pose.unit_quaternion(), pose.translation()));
  v_frame->setId(id);
  v_frame->setFixed(is_fixed);
  return v_frame;
}

g2o_types::VertexPoint* createG2oVertexPoint(
    const MapPoint::Ptr& point, const int id, const bool is_fixed = false,
    const bool is_marginalized = true) {
  auto v_point = new g2o_types::VertexPoint();
  v_point->setEstimate(point->pos());
  v_point->setId(id);
  v_point->setFixed(is_fixed);
  //! This could be skipped if don't use Schur trick (if so, solver type needs
  //! to be changed to another).
  v_point->setMarginalized(is_marginalized);
  return v_point;
}

//! Declared as shared as the edges will be stored in edge container.
g2o_types::EdgeObs* createG2oEdgeObs(
    g2o_types::VertexFrame* v_frame, g2o_types::VertexPoint* v_point,
    const Vec2& pt, const double weight,
    const double huber_delta = std::numeric_limits<double>::infinity()) {
  auto e_obs = new g2o_types::EdgeObs();
  // FIXME How does the memory of VertexContainer in g2o be allocated?
  e_obs->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_frame));
  e_obs->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_point));
  e_obs->setMeasurement(pt);
  e_obs->setInformation(weight * Mat22::Identity());
  g2o::RobustKernelHuber* huber_kernel = new g2o::RobustKernelHuber();
  huber_kernel->setDelta(huber_delta);
  e_obs->setRobustKernel(huber_kernel);
  // Set corresponding camera parameters.
  //! The first parameter 0 is the index of the parameter in the g2o parameters
  //! container. It does not correlate the vertex id.
  // e_obs->setParameterId(0, CAMERA_PARAMETER_ID);
  return e_obs;
}

g2o_types::EdgePoseOnly* createG2oEdgePoseOnly(
    g2o_types::VertexFrame* v_frame, const Vec2& pt, const Vec3& pos,
    const Mat33& K, const double weight,
    const double huber_delta = std::numeric_limits<double>::infinity()) {
  auto e_pose_only = new g2o_types::EdgePoseOnly();
  e_pose_only->setVertex(0,
                         dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_frame));
  e_pose_only->setMeasurement(pt);
  e_pose_only->setInformation(weight * Mat22::Identity());
  auto huber_kernel = new g2o::RobustKernelHuber();
  huber_kernel->setDelta(huber_delta);
  e_pose_only->setRobustKernel(huber_kernel);
  // Set initial pose to speed-up convergence.
  e_pose_only->Xw = pos;
  // Set camera parameters.
  e_pose_only->fx = K(0, 0);
  e_pose_only->fy = K(1, 1);
  e_pose_only->cx = K(0, 2);
  e_pose_only->cy = K(1, 2);
  return e_pose_only;
}

}  // namespace g2o_utils
}  // namespace mono_slam

#endif  // MONO_SLAM_G2O_OPTIMIZER_UTILS_H_