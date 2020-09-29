#ifndef MONO_SLAM_G2O_OPTIMIZER_UTILS_H_
#define MONO_SLAM_G2O_OPTIMIZER_UTILS_H_

#include "mono_slam/common_include.h"
#include "mono_slam/g2o_optimizer/types.h"

namespace g2o_utils {

static void setupG2oOptimizero(const sptr<g2o::SparseOptimizer>& optimizer,
                               const sptr<Camera>& cam) {
  // Set solvers: linear solver and block sovler.
  auto linear_solver = make_unique<g2o_types::LinearSolver>();
  auto block_solver = make_unique<g2o_types::BlockSolver>(linear_solver);
  auto solver = make_unique<g2o::OptimizationAlgorithmLevenberg>(block_solver);
  optimizer->setAlgorithm(solver);
  const Mat33& K = cam->K();
  const double &f = K(0, 0), &cx = K(0, 2), &cy = K(1, 2), &b = 0.;
  auto cam_params = make_unique<g2o::CameraParameters>(f, Vec2{cx, cy}, b);
  cam_params->setId(CAMERA_PARAMETER_ID);
  // FIXME Why do this?
  if (!optimizer->addParameter(cam_params)) {
    assert(false);
  }
}

static void runG2oOptimizer(const sptr<g2o::SparseOptimizer>& optimizer,
                            const int num_iterations) {
  // FIXME Any useful options?
  optimizer->initializeOptimization();
  optimizer->optimize(num_iterations);
}

static inline uptr<VertexFrame> createG2oVertexFrame(
    const Frame::Ptr& keyframe, const int id, const bool is_fixed = false) {
  auto v_frame = make_unique<VertexFrame>();
  const SE3& pose = keyframe->Pose();
  v_frame->setEstimate(g2o::SE3Quat(pose.unitQuaternion(), pose.translation()));
  v_frame->setId(id);
  v_frame->setFixed(is_fixed);
  return v_frame;
}

static inline uptr<VertexPoint> createG2oVertexPoint(
    const MapPoint::Ptr& point, const int id, const bool is_fixed = false,
    const bool is_marginalized = true) {
  auto v_point = make_unique<VertexPoint>();
  v_point->setEstimate(point->Pos());
  v_point->setId(id);
  v_point->setFixed(is_fixed);
  v_point->setMarginalized(is_marginalized);
  return v_point;
}

static inline uptr<EdgeObs> createG2oEdgeObs(
    const sptr<VertexFrame>& v_frame, const sptr<VertexPoint>& v_point,
    const Feature::Ptr& feat, const double weight,
    const double huber_delta = std::numeric_limits<double>::infinity()) {
  auto e_obs = make_unique<EdgeObs>();
  e_obs->setVertex(
      0, dynamic_pointer_cast<g2o::OptimizableGraph::Vertex*>(v_frame));
  e_obs->setVertex(
      1, dynamic_pointer_cast<g2o::OptimizableGraph::Vertex*>(v_point));
  e_obs->setMeasurement(feat->pt_);
  e_obs->setInformation(weight * Mat22::Identity());
  auto huber_kernel = make_unique<g2o::RobustKernelHuber>();
  huber_kernel->setDelta(huber_delta);
  e_obs->setRobustKernel(huber_kernel);
  e_obs->setParameterId(
      1, CAMERA_PARAMETER_ID);  // Apply camera parameters to v_point.
  return e_obs;
}

}  // namespace g2o_utils

#endif  // MONO_SLAM_G2O_OPTIMIZER_UTILS_H_