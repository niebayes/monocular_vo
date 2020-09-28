#ifndef MONO_SLAM_G2O_OPTIMIZER_H_
#define MONO_SLAM_G2O_OPTIMIZER_H_

#include "mono_slam/common_include.h"
#include "mono_slam/g2o_optimizer/types.h"
#include "mono_slam/map.h"

namespace mono_slam {

class Map;

class Optimizer {
 public:
  // Global bundle adjustment.
  static void GlobalBundleAdjustment(const Map::Keyframes& keyframes,
                                     const Map::MapPoints& points, Map::Ptr& map,
                                     const int num_iterations);

  // Pose graph optimization.
  static void PoseGraphOptimization();

  // Local bundle adjustment.
  static void LocalBundleAdjustment();
};

}  // namespace mono_slam

#endif  // MONO_SLAM_G2O_OPTIMIZER_H_