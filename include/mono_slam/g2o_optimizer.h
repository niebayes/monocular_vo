#ifndef MONO_SLAM_G2O_OPTIMIZER_H_
#define MONO_SLAM_G2O_OPTIMIZER_H_

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/types/slam3d/g2o_types_slam3d_api.h"
#include "mono_slam/common_include.h"
#include "mono_slam/g2o_optimizer/types.h"

namespace mono_slam {

class Optimizer {
 public:
  // Global bundle adjustment.
  static void GlobalBundleAdjustment();

  // Pose graph optimization.
  static void PoseGraphOptimization();

  // Local bundle adjustment.
  static void LocalBundleAdjustment();
};

}  // namespace mono_slam

#include "mono_slam/g2o_optimizer/global_bundle_adjustment.h"
#include "mono_slam/g2o_optimizer/local_bundle_adjustment.h"
#include "mono_slam/g2o_optimizer/pose_graph_optimization.h"

#endif  // MONO_SLAM_G2O_OPTIMIZER_H_