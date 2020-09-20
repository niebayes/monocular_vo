#ifndef MY_BACK_END_LOCAL_MAPPING_H_
#define MY_BACK_END_LOCAL_MAPPING_H_

#include "my_slam/common_include.h"
#include "my_slam/front_end_tracking.h"
#include "my_slam/slam_system.h"

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

#endif  // MY_BACK_END_LOCAL_MAPPING_H_