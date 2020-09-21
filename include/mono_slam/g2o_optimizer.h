#ifndef MONO_SLAM_G2O_OPTIMIZER_H_
#define MONO_SLAM_G2O_OPTIMIZER_H_

#include "mono_slam/common_include.h"
#include "mono_slam/g2o_optimizer/types.h"

namespace mono_slam {

class Optimizer {
 public:
  using Ptr = sptr<Optimizer>;

  // Global bundle adjustment.
  static void GlobalBundleAdjustment();

  // Pose graph optimization.
  static void PoseGraphOptimization();

  // Local bundle adjustment.
  static void LocalBundleAdjustment();

 private:
};

}  // namespace mono_slam

#include "mono_slam/g2o_optimizer/global_bundle_adjustment.h"
#include "mono_slam/g2o_optimizer/local_bundle_adjustment.h"
#include "mono_slam/g2o_optimizer/pose_graph_optimization.h"

#endif  // MONO_SLAM_G2O_OPTIMIZER_H_