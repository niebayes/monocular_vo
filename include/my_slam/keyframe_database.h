#ifndef MY_KEYFRAME_DATABASE_H_
#define MY_KEYFRAME_DATABASE_H_

#include "my_slam/common_include.h"

class KeyframeDB {
 public:
  using Ptr = std::unique_ptr<KeyframeDB*>;
};

#endif  // MY_KEYFRAME_DATABASE_H_