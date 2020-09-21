#ifndef MONO_SLAM_FRONT_END_MAP_INITIALIZATION_H_
#define MONO_SLAM_FRONT_END_MAP_INITIALIZATION_H_

#include "mono_slam/common_include.h"
#include "mono_slam/g2o_optimizer.h"
#include "mono_slam/geometry_solver.h"

namespace mono_slam {

class Initializer {
 public:
  using Ptr = sptr<Initializer>;

 private:
};

}  // namespace mono_slam

#endif  // MONO_SLAM_FRONT_END_MAP_INITIALIZATION_H_