#ifndef MONO_SLAM_G2O_OPTIMIZER_UTILS_H_
#define MONO_SLAM_G2O_OPTIMIZER_UTILS_H_

#include "mono_slam/camera.h"
#include "mono_slam/common_include.h"
#include "mono_slam/frame.h"
#include "mono_slam/g2o_optimizer/g2o_types.h"

namespace mono_slam {
namespace g2o_utils {

static void setupG2oOptimizer(const sptr<g2o::SparseOptimizer>& optimizer,
                              const Mat33& K) {
  // Set solver.
  auto solver = g2o::make_unique<g2o::OptimizationAlgorithmLevenberg>(
      g2o::make_unique<g2o_types::BlockSolver>(
          g2o::make_unique<g2o_types::LinearSolver>()));
  optimizer->setAlgorithm(solver.get());
  const double &f = K(0, 0), &cx = K(0, 2), &cy = K(1, 2), &b = 0.;
  auto cam_params = make_unique<g2o::CameraParameters>(f, Vec2{cx, cy}, b);
  cam_params->setId(CAMERA_PARAMETER_ID);
  assert(optimizer->addParameter(cam_params.get()));
}

static void runG2oOptimizer(const sptr<g2o::SparseOptimizer>& optimizer,
                            const int n_iters) {
  // FIXME Any useful options?
  optimizer->initializeOptimization();
  optimizer->optimize(n_iters);
}

static inline sptr<g2o_types::VertexFrame> createG2oVertexFrame(
    const Frame::Ptr& keyframe, const int id, const bool is_fixed = false) {
  auto v_frame = make_unique<g2o_types::VertexFrame>();
  const SE3& pose = keyframe->pose();
  v_frame->setEstimate(
      g2o::SE3Quat(pose.unit_quaternion(), pose.translation()));
  v_frame->setId(id);
  v_frame->setFixed(is_fixed);
  return v_frame;
}

static inline sptr<g2o_types::VertexPoint> createG2oVertexPoint(
    const MapPoint::Ptr& point, const int id, const bool is_fixed = false,
    const bool is_marginalized = true) {
  auto v_point = make_unique<g2o_types::VertexPoint>();
  v_point->setEstimate(point->pos());
  v_point->setId(id);
  v_point->setFixed(is_fixed);
  //! This could be skipped if not using Schur trick (if so, solver type needs
  //! to be changed to another).
  v_point->setMarginalized(is_marginalized);
  return v_point;
}

static inline sptr<g2o_types::EdgeObs> createG2oEdgeObs(
    const sptr<g2o_types::VertexFrame>& v_frame,
    const sptr<g2o_types::VertexPoint>& v_point, const Vec2& pt,
    const double weight,
    const double huber_delta = std::numeric_limits<double>::infinity()) {
  auto e_obs = make_unique<g2o_types::EdgeObs>();
  // FIXME How does the memory of VertexContainer in g2o be allocated?
  // FIXME Is there errors if I first set vertex 0 and then vertex 1?
  e_obs->setVertex(1,
                   dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_frame.get()));
  e_obs->setVertex(0,
                   dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_point.get()));
  e_obs->setMeasurement(pt);
  e_obs->setInformation(weight * Mat22::Identity());
  auto huber_kernel = make_unique<g2o::RobustKernelHuber>();
  huber_kernel->setDelta(huber_delta);
  e_obs->setRobustKernel(huber_kernel.get());
  // Set corresponding camera parameters.
  //! The first parameter 0 is the index of the parameter in the g2o parameters
  //! container. It does not correlate the vertex id.
  e_obs->setParameterId(0, CAMERA_PARAMETER_ID);
  return e_obs;
}

static inline sptr<g2o_types::EdgePoseOnly> createG2oEdgePoseOnly(
    const sptr<g2o_types::VertexFrame>& v_frame, const Vec2& pt,
    const Vec3& pos, const Mat33& K, const double weight,
    const double huber_delta = std::numeric_limits<double>::infinity()) {
  auto e_pose_only = make_unique<g2o_types::EdgePoseOnly>();
  e_pose_only->setVertex(
      0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_frame.get()));
  e_pose_only->setMeasurement(pt);
  e_pose_only->setInformation(weight * Mat22::Identity());
  auto huber_kernel = make_unique<g2o::RobustKernelHuber>();
  huber_kernel->setDelta(huber_delta);
  e_pose_only->setRobustKernel(huber_kernel.get());
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