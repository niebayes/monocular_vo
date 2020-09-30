#ifndef MONO_SLAM_MAP_H_
#define MONO_SLAM_MAP_H_

#include "mono_slam/common_include.h"
#include "mono_slam/frame.h"
#include "mono_slam/map_point.h"

namespace mono_slam {

class Frame;
class MapPoint;

class Map {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = sptr<Map>;

  void InsertKeyframe(Frame::Ptr keyframe);

  void EraseKeyframeById(const int id);

  // FIXME Return copy or const reference?
  inline const list<Frame::Ptr>& GetAllKeyframes() {
    u_lock lock(mutex_);
    return keyframes_;
  }

  // FIXME Candidate points?
  inline const list<MapPoint::Ptr>& GetAllMapPoints() {
    u_lock lock(mutex_);
    return points_;
  }

  inline void Clear() {
    u_lock lock(mutex_);
    keyframes_.clear();
    points_.clear();
    max_frame_id_ = 0;
  }

  // TODO(bayes) Implement remove functions, e.g. put outlier map points to
  // trash and empty trash properly. And more function like svo.

 private:
  list<Frame::Ptr> keyframes_;  // Maintained keyframes.
  list<MapPoint::Ptr> points_;  // Maintained map points.

  // FIXME Would it be better if we simply use global frame counter?
  int max_frame_id_;  // Maximum id of frames inserted so far. Used for
                      // checking for duplication as new keyframe is comming.

  mutable std::mutex mutex_;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_MAP_H_