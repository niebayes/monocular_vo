#include "mono_slam/frame.h"

#include "mono_slam/feature.h"

namespace mono_slam {

Frame(const cv::Mat& img, const Camera::Ptr& cam, const sptr<Vocabulary>& voc,
      const cv::Ptr<cv::FeatureDetector>& detector)
    : id_(frame_cnt_++), is_keyframe_(false), cam_(cam) {
  ExtractFeatures(img, detector);
  ComputeBoW(voc);
  // TODO(bayes) Optimize when no distortion.
  // Compute image bounds (computed once in the first frame).
  if (id_ == 0) {
    // Matrix containing the four corners of the image:
    // Left upper, right upper, left bottom, right bottom.
    cv::Mat corners;
    frame_utils::ComputeImageBounds(img, cam_->K(), cam_->DistCoeffs(),
                                    corners);
    x_min_ = std::min(corners.at<float>(0, 0), corners.at<float>(2, 0));
    x_max_ = std::max(corners.at<float>(1, 0), corners.at<float>(3, 0));
    y_min_ = std::min(corners.at<float>(0, 1), corners.at<float>(1, 1));
    y_max_ = std::max(corners.at<float>(2, 1), corners.at<float>(3, 1));
  }
}

void Frame::SetPose(const SE3& T_c_w) { cam_->SetPose(T_c_w); }

void Frame::SetPos(const Vec3& pos) { cam_->SetPos(pos); }

void Frame::SetKeyframe() { is_keyframe_ = true; }

void Frame::ExtractFeatures(const cv::Mat& img,
                            const cv::Ptr<cv::FeatureDetector>& detector) {
  vector<cv::KeyPoint> kpts;
  cv::Mat descriptors;
  detector->detectAndCompute(img, cv::Mat{}, kpts, descriptors);
  if (!cam_->DistCoeffs().empty())
    frame_utils::UndistortKeypoints(cam_->K(), cam_->DistCoeffs(), kpts);
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

vector<int> Frame::SearchFeatures(const Vec2& pt, const int radius,
                                  const int level_low, const int level_high) {
  CHECK(level_low >= 0 && level_high >= level_low);
  const int num_obs = this->NumObs();
  vector<int> feat_indices;
  feat_indices.reserve(num_obs);
  for (int i = 0; i < num_obs; ++i) {
    const sptr<Feature>& feat = feats_[i];
    const Vec2 dist = (feat->pt_ - pt).abs();
    if (dist.x() <= radius && dist.y() <= radius)
      if (feat->level >= level_low && feat->level <= level_high)
        feat_indices.push_back(i);
  }
  return feat_indices;
}
namespace frame_utils {

void UndistortKeypoints(const Mat33& K, const Vec4& dist_coeffs,
                        std::vector<cv::KeyPoint>& kpts) {
  const int num_kpts = kpts.size();
  cv::Mat kpts_mat(num_kpts, 2, CV_64F);
  for (int i = 0; i < num_kpts; ++i) {
    kpts_mat.at<double>(i, 0) = kpts[i].pt.x;
    kpts_mat.at<double>(i, 1) = kpts[i].pt.y;
  }
  cv::Mat K_, dist_coeffs_;
  cv::cv2eigen(K, K_);
  cv::cv2eigen(dist_coeffs, dist_coeffs_);
  cv::undistortPoints(kpts_mat, kpts_mat, K_, dist_coeffs_, cv::Mat{}, K_);
  for (int i = 0; i < num_kpts; ++i) {
    kpts[i].pt.x = kpts_mat.at<double>(i, 0);
    kpts[i].pt.y = kpts_mat.at<double>(i, 1);
  }
}

void ComputeImageBounds(const cv::Mat& img, const Mat33& K,
                        const Vec4& dist_coeffs, cv::Mat& corners) {
  corners = cv::Mat(4, 2, CV_32F);
  cv::Mat K_, dist_coeffs_;
  cv::cv2eigen(K, K_);
  cv::cv2eigen(dist_coeffs, dist_coeffs_);
  corners.at<float>(0, 0) = 0.0;
  corners.at<float>(0, 1) = 0.0;
  corners.at<float>(1, 0) = img.cols;
  corners.at<float>(1, 1) = 0.0;
  corners.at<float>(2, 0) = 0.0;
  corners.at<float>(2, 1) = img.rows;
  corners.at<float>(3, 0) = img.cols;
  corners.at<float>(3, 1) = img.rows;
  cv::undistortPoints(corners, corners, K_, dist_coeffs_, cv::Mat{}, K_);
}

}  // namespace frame_utils
}  // namespace mono_slam