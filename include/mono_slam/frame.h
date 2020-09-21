#ifndef MONO_SLAM_FRAME_H_
#define MONO_SLAM_FRAME_H_

#include "mono_slam/common_include.h"
#include "mono_slam/feature.h"
#include "mono_slam/map_point.h"

namespace mono_slam {

class Frame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = sptr<Frame>;

 private:
};

}  // namespace mono_slam

#endif  // MONO_SLAM_FRAME_H_