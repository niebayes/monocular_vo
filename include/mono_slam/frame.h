#ifndef MONO_SLAM_FRAME_H_
#define MONO_SLAM_FRAME_H_

#include "mono_slam/camera.h"
#include "mono_slam/common_include.h"
#include "mono_slam/feature.h"
#include "mono_slam/map_point.h"

namespace mono_slam {

struct Feature;
class Camera;

class Frame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = sptr<Frame>;
  using Features = std::vector<sptr<Feature>>;

  Frame();

  static inline void CreateFrame();

  inline int NumFeatures() { return feats_.size(); }

 public:
  // Global frame counter, starting from 0.
  static int frame_cnt_;
  // Frame identity.
  int id_;
  // Is this frame a keyframe?
  bool is_keyframe_;

  // Features extracted in the image.
  Features feats_;
  // Linked Camera.
  Camera::Ptr cam_;

  // Image bounds.
  static double x_min_;
  static double x_max_;
  static double y_min_;
  static double y_max_;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_FRAME_H_