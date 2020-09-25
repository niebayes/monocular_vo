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

  Feature(const wptr<Frame>& frame, const Vec2& pt, const cv::Mat& descriptor,
          const int level)
      : frame_(frame), pt_(pt), descriptor_(descriptor), level_(level), bear_vec_(frame_->cam_->pixel2bear(pt_)  {}

 public:
  // Linked 3D map point expressed in world frame.
  wptr<MapPoint> point_;

  // Feature characteristics.
  const wptr<Frame> frame_;   // Frame in which the feature is detected.
  const Vec2 pt_;             // 2D image point expressed in pixels.
  const cv::Mat descriptor_;  // Corresponding descriptor.
  const Vec3 bear_vec_;  // Unit bearing vector (i.e. the ray through the camera
                         // center of frame_ and pt_)
  const int level_;  // Image pyramid level at which the feature is detected.
};

}  // namespace mono_slam

#endif  // MONO_SLAM_FEATURE_H_