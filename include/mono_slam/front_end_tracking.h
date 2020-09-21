#ifndef MONO_SLAM_FRONT_END_TRACKING_H_
#define MONO_SLAM_FRONT_END_TRACKING_H_

#include "mono_slam/back_end_local_mapping.h"
#include "mono_slam/common_include.h"
#include "mono_slam/slam_system.h"

namespace mono_slam {

class LocalMapping;
class System;

class Tracking {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = sptr<Tracking>;
  // weark2shared

  Tracking() {}

  void SetLocalMapper(sptr<LocalMapping> local_mapper) {
    local_mapper_ = local_mapper;
  }

 private:
  sptr<LocalMapping> local_mapper_;
};

}  // namespace mono_slam

#endif  // MY_FRONT_END_TRACKING