#ifndef MONO_SLAM_MATCHER_H_
#define MONO_SLAM_MATCHER_H_

#include "mono_slam/common_include.h"
#include "mono_slam/feature.h"
#include "mono_slam/frame.h"

namespace mono_slam {

namespace matcher_utils {

//@ref http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int ComputeDescriptorDistance(const cv::Mat& desc_1, const cv::Mat& desc_2);
}  // namespace matcher_utils

class Matcher {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Matcher(const int matching_threshold,
          const int distance_ratio_test_threshold);

  // Return number of matches.
  static int SearchForInitialization(const Frame::Ptr& frame_1,
                                     const Frame::Ptr& frame_2,
                                     vector<int>& matches);

 private:
  const int matching_threshold_;
  const int distance_ratio_test_threshold_;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_MATCHER_H_