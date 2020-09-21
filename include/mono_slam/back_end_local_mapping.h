#ifndef MONO_SLAM_BACK_END_LOCAL_MAPPING_H_
#define MONO_SLAM_BACK_END_LOCAL_MAPPING_H_

#include "mono_slam/common_include.h"
#include "mono_slam/front_end_tracking.h"
#include "mono_slam/slam_system.h"

namespace mono_slam {

class Tracking;
class System;

class LocalMapping {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<LocalMapping>;

  LocalMapping() {}

  void SetTracker(sptr<Tracking> tracker) { tracker_ = tracker; }

 private:
  sptr<Tracking> tracker_;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_BACK_END_LOCAL_MAPPING_H_