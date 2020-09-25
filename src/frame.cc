#include "mono_slam/frame.h"

namespace mono_slam {

Frame(const cv::Mat& img, const Camera::Ptr& cam, const sptr<Vocabulary>& voc,
      const cv::Ptr<cv::FeatureDetector>& detector)
    : id_(frame_cnt_++), is_keyframe_(false), cam_(cam) {
  ExtractFeatures(img, detector);
  ComputeBoW(voc);
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

void Frame::ExtractFeatures(const cv::Mat& img,
                            const cv::Ptr<cv::FeatureDetector>& detector) {
  vector<cv::KeyPoint> kpts;
  cv::Mat descriptors;
  detector->detectAndCompute(img, cv::Mat{}, kpts, descriptors);
  const int num_kpts = kpts.size();
  feats_.reserve(num_kpts);
  for (int i = 0; i < num_kpts; ++i) {
    feats_.push_back(make_shared<Feature>(this,
                                          Vec2{kpts[i].pt.x, kpts[i].pt.y},
                                          descriptors[i], kpts[i].octave));
  }
}

void Frame::ComputeBoW(const sptr<Vocabulary>& voc) {
  // Collect descriptors.
  vector<cv::Mat> descriptor_vec;
  descriptor_vec.reserve(this->NumObservations());
  std::transform(feats_.cbegin(), feats_.cend(),
                 std::back_inserter(descriptor_vec),
                 [](sptr<Feature> feat) { return feat->descriptor_; });
  voc->transform(descriptor_vec, bow_vec_, feat_vec_, 4);
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