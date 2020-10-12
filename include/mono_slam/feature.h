#ifndef MONO_SLAM_FEATURE_H_
#define MONO_SLAM_FEATURE_H_

#include "mono_slam/common_include.h"
#include "mono_slam/frame.h"
#include "mono_slam/map_point.h"

namespace mono_slam {

class Frame;
class MapPoint;

struct Feature {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = sptr<Feature>;

  // weak_ptr to avoid cyclic reference which makes memory release unaviable.
  const wptr<Frame> frame_;   // Frame in which the feature is detected.
  const Vec2 pt_;             // 2D image point expressed in pixels.
  const cv::Mat descriptor_;  // Corresponding descriptor.
  const int level_;  // Image pyramid level at which the feature is detected.
  // FIXME Should feature be deleted immediately?
  // Linked 3D map point expressed in world frame.
  //! A feature links only one map point and will not change any more once set.
  wptr<MapPoint> point_;
  bool is_outlier_;  // Is the observation formed with this feature and the
                     // point_ an outlier?

  Feature(sptr<Frame> frame, const Vec2& pt, const cv::Mat& descriptor,
          const int level)
      : frame_(frame),
        pt_(pt),
        descriptor_(descriptor),
        level_(level),
        is_outlier_(false) {}
};

namespace feat_utils {

// FIXME static and forward declaration conflict with each other?

// FIXME Would it be okay if these two utility functions being member methods?
static inline sptr<MapPoint> getPoint(const sptr<Feature>& feat) {
  if (feat == nullptr || feat->is_outlier_) return nullptr;
  const sptr<MapPoint>& point = feat->point_.lock();
  // if (point->to_be_deleted_) return nullptr;
  return point;
}

static inline sptr<Frame> getKeyframe(const sptr<Feature>& feat) {
  if (!feat || feat->is_outlier_) return nullptr;
  if (feat->frame_.expired()) return nullptr;
  const sptr<Frame>& keyframe = feat->frame_.lock();
  // FIXME What fk causes this error?!
  // if (!keyframe->isKeyframe()) return nullptr;
  return keyframe;
}

}  // namespace feat_utils
}  // namespace mono_slam

#endif  // MONO_SLAM_FEATURE_H_