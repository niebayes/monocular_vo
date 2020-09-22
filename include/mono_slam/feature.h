#ifndef MONO_SLAM_FEATURE_H_
#define MONO_SLAM_FEATURE_H_

#include "mono_slam/common_include.h"
#include "mono_slam/frame.h"
#include "mono_slam/map_point.h"

namespace mono_slam {

class Frame;
class MapPoint;
struct Feature {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = sptr<Feature>;

 public:
  // Frame in which the feature is extracted.
  wptr<Frame> frame_;
  // 2D image point expressed in pixels.
  Vec2 pt_;
  // 3D scene (map) point in world frame.
  wptr<MapPoint> point_;
  // Unit bearing vector (i.e. the ray through the camera
  // center and the image point).
  Vec3 bear_vec_;
  // Image pyramid level at which the feature is detected. If the feature
  // detector is not scale-perceivable, it's set to 0.
  int level;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_FEATURE_H_