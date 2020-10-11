#ifndef MONO_SLAM_FRAME_H_
#define MONO_SLAM_FRAME_H_

#include "DBoW3/DBoW3.h"
#include "mono_slam/camera.h"
#include "mono_slam/common_include.h"
#include "mono_slam/feature.h"
#include "mono_slam/g2o_optimizer/g2o_types.h"
#include "mono_slam/map_point.h"

namespace mono_slam {

class Camera;
struct Feature;
class MapPoint;

class Frame : public std::enable_shared_from_this<Frame> {
 public:
  using Ptr = sptr<Frame>;
  // FIXME Use unique_ptr for features since they're exclusively owned by frame.
  using Features = vector<sptr<Feature>>;

  // FIXME Should frame has a member denoting self a frame to be deleted?

  static int frame_cnt_;           // Global frame counter, starting from 0.
  const int id_;                   // Unique frame identity.
  bool is_keyframe_;               // Is this frame a keyframe?
  Features feats_;                 // Features extracted in this frame.
  Camera::Ptr cam_{nullptr};      // Linked camera.
  DBoW3::BowVector bow_vec_;       // Bag of words vector.
  DBoW3::FeatureVector feat_vec_;  // Feature vector.

  // Temporary variables used for relocalization.
  int query_frame_id_;     // Id of currently quering frame.
  int n_sharing_words_;    // Number of sharing words between this and currently
                           // quering frame.
  double bow_simi_score_;  // Similarity score between the bag-of-words vector
                           // of this and that of currently quering frame.
  bool is_candidate_already_;  // Is this keyframe selected as relocalization
                               // candidate already?

  // Temporary g2o keyframe vertex storing the optimized result.
  //! No memeory leak since it's freed as the g2o::OptimizableGraph is cleared.
  g2o_types::VertexFrame* v_frame_{nullptr};

  // Variables used for covisibility graph.
  unordered_map<Frame::Ptr, int> co_kf_weights_;
  forward_list<Frame::Ptr> co_kfs_;
  forward_list<int> co_weights_;

  // Image bounds.
  static double x_min_;
  static double x_max_;
  static double y_min_;
  static double y_max_;

  Frame(const cv::Mat& img);

  inline const SE3& pose() const {
    lock_g lock(mut_);
    return cam_->pose();
  }

  void setPose(const SE3& T_c_w);

  inline bool isKeyframe() const {
    lock_g lock(mut_);
    return is_keyframe_;
  }

  void setKeyframe();

  // Number of observations (i.e. number of features observed in this frame).
  inline int nObs() const {
    lock_g lock(mut_);
    return static_cast<int>(feats_.size());
  }

  // Search features given searching radius and image pyramid level range.
  vector<int> searchFeatures(const Vec2& pt, const int radius,
                             const int level_low, const int level_high) const;

  // Check if the given map point is Observable by this frame.
  bool isObservable(const sptr<MapPoint>& point, const int level) const;

  void addConnection(Frame::Ptr keyframe, const int weight);

  void deleteConnection(const Frame::Ptr& keyframe);

  void updateCoKfsAndWeights();

  void updateCoInfo();

  double computeSceneMedianDepth();

  inline forward_list<Frame::Ptr> getCoKfs(
      const int n = std::numeric_limits<int>::max()) const {
    lock_g lock(co_mut_);
    const int n_kfs =
        static_cast<int>(std::distance(co_kfs_.cbegin(), co_kfs_.cend()));
    if (n > n_kfs) return co_kfs_;
    return forward_list<Frame::Ptr>(co_kfs_.cbegin(),
                                    std::next(co_kfs_.cbegin(), n));
  }

  // FIXME This method is deprecated!
  // Compute number of tracked map points (i.e. ones that are observed by more
  // than min_n_obs frames).
  int computeTrackedPoints(const int min_n_obs) const;

  // FIXME Personally, this method should be in map.
  // Erase the links between this frame and other stuff.
  void erase();

 private:
  // Mutexes.
  mutable std::mutex mut_;  // General data guardian.
  // Protect concurrent modification on covisible info.
  mutable std::mutex co_mut_;
};

namespace frame_utils {

void undistortKeypoints(const Mat33& K, const Vec4& dist_coeffs,
                        std::vector<cv::KeyPoint>& kpts);

void computeImageBounds(const cv::Mat& img, const Mat33& K,
                        const Vec4& dist_coeffs, cv::Mat& corners);

}  // namespace frame_utils
}  // namespace mono_slam

#endif  // MONO_SLAM_FRAME_H_