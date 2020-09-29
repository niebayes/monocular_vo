#ifndef MONO_SLAM_FRAME_H_
#define MONO_SLAM_FRAME_H_

#include "mono_slam/camera.h"
#include "mono_slam/common_include.h"
#include "mono_slam/feature.h"
#include "mono_slam/g2o_optimizer/types.h"
#include "mono_slam/map_point.h"

namespace mono_slam {

class Camera;
struct Feature;
class MapPoint;

class Frame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = sptr<Frame>;
  using Features = std::vector<wptr<Feature>>;  // weak_ptr to avoid cyclic ref.

  static int frame_cnt_;           // Global frame counter, starting from 0.
  const int id_;                   // Unique frame identity.
  bool is_keyframe_;               // Is this frame a keyframe?
  Features feats_;                 // Features extracted in this frame.
  Camera::Ptr cam_ = nullptr;      // Linked camera.
  DBoW3::BowVector bow_vec_;       // Bag of words vector.
  DBoW3::FeatureVector feat_vec_;  // Feature vector.
  sptr<g2o_types::VertexFrame> v_frame_ =
      nullptr;  // Temporary g2o keyframe vertex storing the optimized result.

  Frame(const cv::Mat& img, Camera::Ptr cam, const sptr<Vocabulary>& voc,
        const cv::Ptr<cv::FeatureDetector>& detector);

  inline const SE3& pose() const { return cam_->pose(); }

  void setPose(const SE3& T_c_w);

  inline const Vec3& pos() const { return cam_->pos(); }

  void setPos(const Vec3& pos);

  inline bool isKeyframe() const { return is_keyframe_; }

  void setKeyframe();

  // Number of observations (i.e. number of features observed in this frame).
  inline int numObs() const { return feats_.size(); }

  // Extract features.
  void extractFeatures(const cv::Mat& img,
                       const cv::Ptr<cv::FeatureDetector>& detector);

  // Compute bag of words representation.
  void computeBoW(const sptr<Vocabulary>& voc);

  // Search features given searching radius and image pyramid level range.
  vector<int> searchFeatures(const Vec2& pt, const int radius,
                             const int level_low, const int level_high) const;

  // Check if the given map point is Observable by this frame.
  bool isObservable(const sptr<MapPoint>& point) const;

  void updateConnections();

  double computeSceneMedianDepth();

 private:
  // Image bounds.
  static double x_min_;
  static double x_max_;
  static double y_min_;
  static double y_max_;
};

namespace frame_utils {

void UndistortKeypoints(const Mat33& K, const Vec4& dist_coeffs,
                        std::vector<cv::KeyPoint>& kpts);

void ComputeImageBounds(const cv::Mat& img, const Mat33& K,
                        const Vec4& dist_coeffs, cv::Mat& corners);

}  // namespace frame_utils
}  // namespace mono_slam

#endif  // MONO_SLAM_FRAME_H_