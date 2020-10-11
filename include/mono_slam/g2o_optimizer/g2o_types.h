#ifndef MONO_SLAM_G2O_OPTIMIZER_TYPES_H_
#define MONO_SLAM_G2O_OPTIMIZER_TYPES_H_

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/solver.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/types/sba/types_sba.h"  // g2o::VertexSBAPointXYZ
#include "g2o/types/sba/types_six_dof_expmap.h"  // g2o::VertexSE3Expmap, g2o::EdgeProjectXYZ2UV, g2o::EdgeSE3ProjectXYZOnlyPose
#include "mono_slam/common_include.h"
#include "mono_slam/feature.h"
#include "mono_slam/frame.h"

namespace mono_slam {

struct Feature;
class Frame;

namespace g2o_types {

// typedefs for solvers.
using BlockSolver = g2o::BlockSolver_6_3;
using LinearSolver = g2o::LinearSolverCholmod<BlockSolver::PoseMatrixType>;

// Edge and vertex typedefs.
using EdgeObs = g2o::EdgeSE3ProjectXYZ;
using EdgePoseOnly = g2o::EdgeSE3ProjectXYZOnlyPose;
using VertexFrame = g2o::VertexSE3Expmap;
using VertexPoint = g2o::VertexSBAPointXYZ;

// FIXME unique_ptr incurs error here.
struct EdgeContainer {
  //! No memeory leak since it's freed as g2o::OptimizableGraph is cleared.
  EdgeObs* e_obs_{nullptr};
  sptr<Frame> keyframe_{nullptr};
  sptr<Feature> feat_{nullptr};
  EdgeContainer(EdgeObs* e_obs, sptr<Frame> keyframe, sptr<Feature> feat)
      : e_obs_(e_obs), keyframe_(keyframe), feat_(feat) {}
};

struct EdgeContainerPoseOnly {
  //! No memeory leak since it's freed as g2o::OptimizableGraph is cleared.
  EdgePoseOnly* e_pose_only_{nullptr};
  sptr<Feature> feat_{nullptr};
  EdgeContainerPoseOnly(EdgePoseOnly* e_pose_only, sptr<Feature> feat)
      : e_pose_only_(e_pose_only), feat_(feat) {}
};

}  // namespace g2o_types
}  // namespace mono_slam

#endif  // MONO_SLAM_G2O_OPTIMIZER_TYPES_H_