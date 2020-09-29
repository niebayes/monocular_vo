#ifndef MONO_SLAM_MATCHER_H_
#define MONO_SLAM_MATCHER_H_

#include "mono_slam/common_include.h"
#include "mono_slam/frame.h"

namespace mono_slam {

class Matcher {
 public:
  // Search feature correspondences between the two views used for
  // initialization.
  static int searchForInitialization(const Frame::Ptr& frame_1,
                                     const Frame::Ptr& frame_2,
                                     vector<int>& matches);

  static int searchByProjection(const std::set<Frame::Ptr>& frames,
                                const Frame::Ptr& curr_frame);
                                
  static int searchByProjection(const Frame::Ptr& last_frame,
                                const Frame::Ptr& curr_frame);
};

namespace matcher_utils {

//@ref http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int computeDescriptorDistance(const cv::Mat& desc_1, const cv::Mat& desc_2);

}  // namespace matcher_utils
}  // namespace mono_slam

#endif  // MONO_SLAM_MATCHER_H_