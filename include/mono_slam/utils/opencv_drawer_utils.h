#ifndef MONO_SLAM_UTILS_OPENCV_DRAWER_UTILS_H_
#define MONO_SLAM_UTILS_OPENCV_DRAWER_UTILS_H_

#include <utility>  // std:pair
#include <vector>

#include "mono_slam/frame.h"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

namespace mono_slam {

class Frame;
namespace viewer_utils {

class OpencvDrawer {
 public:
  static void drawMatches(const Frame::Ptr& ref_frame,
                          const Frame::Ptr& curr_frame,
                          const vector<pair<int, int>>& inlier_matches,
                          cv::Mat& img_show);

  static void drawKeyPoints(const Frame::Ptr& frame, cv::Mat& img_show);
};

namespace opencv_utils {
static void pts2kpts(const Frame::Ptr& frame, vector<cv::KeyPoint>& kpts);
}  // namespace opencv_utils
}  // namespace viewer_utils
}  // namespace mono_slam

#endif  // MONO_SLAM_UTILS_OPENCV_DRAWER_UTILS_H_