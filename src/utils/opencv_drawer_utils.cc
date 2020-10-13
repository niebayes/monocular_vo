#include "mono_slam/utils/opencv_drawer_utils.h"

namespace viewer_utils {

void OpencvDrawer::drawMatches(const Frame::Ptr& ref_frame,
                               const Frame::Ptr& curr_frame,
                               const vector<pair<int, int>>& inlier_matches,
                               cv::Mat& img_show) {
  vector<cv::KeyPoint> ref_kpts, curr_kpts;
  opencv_utils::pts2kpts(ref_frame, ref_kpts);
  opencv_utils::pts2kpts(curr_frame, curr_kpts);
  // Transform pairs of matches to vector of DMatches.
  // Inlier matches from ref_frame to curr_frame.
  vector<cv::DMatch> matches_ref_curr;
  matches_ref_curr.reserve(inlier_matches.size());
  std::transform(inlier_matches.cbegin(), inlier_matches.cend(),
                 matches_ref_curr.begin(), [](const pair<int, int>& match) {
                   return cv::DMatch(match.fisrt, match.second, 0.0f);
                 });
  cv::drawMatches(ref_frame->img_, ref_kpts, curr_frame->img_, curr_kpts,
                  matches_ref_curr, img_show, {255, 0, 0}, {0, 255, 0});
}

void OpencvDrawer::drawKeyPoints(const Frame::Ptr& frame, cv::Mat& img_show) {
  vector<cv::KeyPoint> kpts;
  opencv_utils::pts2kpts(frame, kpts);
  cv::drawKeypoints(frame->img_, kpts, img_show, {0, 255, 0});
}

namespace opencv_utils {

void pts2kpts(const Frame::Ptr& frame, vector<cv::KeyPoint>& kpts) {
  kpts.reserve(frame->feats_.size());
  for (const Feature::Ptr& feat : frame->feats_)
    kpts.push_back(cv::KeyPoint(feat->pt_.x(), feat->pt_.y(), 0.0f));
}

}  // namespace opencv_utils
}  // namespace viewer_utils
