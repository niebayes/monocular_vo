#include "mono_slam/frame.h"

namespace mono_slam {

Frame::Frame(const cv::Mat& img)
    : id_(frame_cnt_++), is_keyframe_(false), cam_(make_unique<Camera>()) {
  // Set all feature pointers to nullptr.
  std::fill(feats_.begin(), feats_.end(), nullptr);

  // Compute image bounds (computed once in the first frame).
  if (id_ == 0) {
    // Matrix containing the four corners of the image:
    // Left upper, right upper, left bottom, right bottom.
    cv::Mat corners;
    cv::Mat K, DistCoeffs;
    cv::eigen2cv(cam_->K(), K);
    cv::eigen2cv(cam_->DistCoeffs(), DistCoeffs);
    frame_utils::ComputeImageBounds(img, K, DistCoeffs, corners);
    x_min_ = std::min(corners.at<float>(0, 0), corners.at<float>(2, 0));
    x_max_ = std::max(corners.at<float>(1, 0), corners.at<float>(3, 0));
    y_min_ = std::min(corners.at<float>(0, 1), corners.at<float>(1, 1));
    y_max_ = std::max(corners.at<float>(2, 1), corners.at<float>(3, 1));
  }
}

namespace frame_utils {

void ComputeImageBounds(const cv::Mat& img, const cv::Mat& K,
                        const cv::Mat& DistCoeffs, cv::Mat& corners) {
  corners = cv::Mat(4, 2, CV_32F);
  corners.at<float>(0, 0) = 0.0;
  corners.at<float>(0, 1) = 0.0;
  corners.at<float>(1, 0) = img.cols;
  corners.at<float>(1, 1) = 0.0;
  corners.at<float>(2, 0) = 0.0;
  corners.at<float>(2, 1) = img.rows;
  corners.at<float>(3, 0) = img.cols;
  corners.at<float>(3, 1) = img.rows;
  cv::undistortPoints(corners, corners, K, DistCoeffs, cv::Mat{}, K);
}

}  // namespace frame_utils
}  // namespace mono_slam