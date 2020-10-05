#ifndef MONO_SLAM_G2O_OPTIMIZER_H_
#define MONO_SLAM_G2O_OPTIMIZER_H_

#include "mono_slam/frame.h"
#include "mono_slam/map.h"
#include "mono_slam/g2o_optimizer/g2o_types.h"

namespace mono_slam {

class Frame;
class Map;

class Optimizer {
 public:
  // Global bundle adjustment.
  static void globalBA(const Map::Ptr& map, const int n_iters = 20);

  // Pose graph optimization.
  static int optimizePose(const Frame::Ptr& frame, const int n_iters = 10);

  // Local bundle adjustment.
  static void localBA(const Frame::Ptr& keyframe, const Map::Ptr& map,
                      const int n_iters = 5);
};

}  // namespace mono_slam

#endif  // MONO_SLAM_G2O_OPTIMIZER_H_