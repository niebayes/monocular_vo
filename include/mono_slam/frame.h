#ifndef MONO_SLAM_FRAME_H_
#define MONO_SLAM_FRAME_H_

#include "mono_slam/camera.h"
#include "mono_slam/common_include.h"
#include "mono_slam/feature.h"

namespace mono_slam {

struct Feature;
class Camera;

class Frame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = sptr<Frame>;
  using Features = std::vector<sptr<Feature>>;

  Frame(const cv::Mat& img, const Camera::Ptr& cam, const sptr<Vocabulary>& voc,
        const cv::Ptr<cv::FeatureDetector>& detector);

  inline SE3 pose() const { return cam_->pose(); }

  void SetPose(const SE3& T_c_w) { cam_->SetPose(T_c_w); }

  void SetKeyframe() { is_keyframe_ = true; }
 
  // Number of observations (i.e. number of features observed in this frame).
  inline int NumObs() const { return feats_.size(); }

  // Extract features.
  void ExtractFeatures(const cv::Mat& img,
                       const cv::Ptr<cv::FeatureDetector>& detector);

  // Compute bag of words representation.
  void ComputeBoW(const sptr<Vocabulary>& voc);

  // Search features given searching radius and image pyramid level range.
  vector<int> SearchFeatures(const Vec2& pt, const int radius,
                             const int level_low, const int level_high);

 public:
  static int frame_cnt_;  // Global frame counter, starting from 0.
  const int id_;          // Unique frame identity.
  bool is_keyframe_;      // Is this frame a keyframe?

  // Frame characteristics.
  const Features feats_;                 // Features extracted in this frame.
  Camera::Ptr cam_ = nullptr;            // Linked camera.
  const DBoW3::BowVector bow_vec_;       // Bag of words vector.
  const DBoW3::FeatureVector feat_vec_;  // Feature vector.

  // Image bounds.
  static double x_min_;
  static double x_max_;
  static double y_min_;
  static double y_max_;
};

namespace frame_utils {

void UndistortKeypoints(std::vector<cv::KeyPoint>& kpts);

void ComputeImageBounds(const cv::Mat& img, const cv::Mat& K,
                        const cv::Mat& DistCoeffs, cv::Mat& corners);
}  // namespace frame_utils

}  // namespace mono_slam

#endif  // MONO_SLAM_FRAME_H_