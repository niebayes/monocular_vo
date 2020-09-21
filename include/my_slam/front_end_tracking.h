#ifndef MY_FRONT_END_TRACKING_H_
#define MY_FRONT_END_TRACKING_H_

#include "my_slam/back_end_local_mapping.h"
#include "my_slam/common_include.h"
#include "my_slam/slam_system.h"

class LocalMapping;
class System;

class Tracking {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Tracking>;
  // weark2shared

  Tracking() {}

  void SetLocalMapper(sptr<LocalMapping> local_mapper) {
    local_mapper_ = local_mapper;
  }

 private:
  sptr<LocalMapping> local_mapper_;
};

#endif  // MY_FRONT_END_TRACKING