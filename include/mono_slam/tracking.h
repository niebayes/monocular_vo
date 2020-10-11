#ifndef MONO_SLAM_TRACKING_H_
#define MONO_SLAM_TRACKING_H_

#include "mono_slam/common_include.h"
#include "mono_slam/frame.h"
#include "mono_slam/initialization.h"
#include "mono_slam/local_mapping.h"
#include "mono_slam/map.h"
#include "mono_slam/system.h"
#include "mono_slam/viewer.h"
#include "DBoW3/DBoW3.h"

using DBoW3::Vocabulary;

namespace mono_slam {

class System;
class LocalMapping;
class Map;
class Frame;
class Viewer;
class Initializer;

// FIXME RESETTING state seems redundant.
enum class State { NOT_INITIALIZED_YET, GOOD, LOST };

class Tracking : public std::enable_shared_from_this<Tracking> {
 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  State state_;                      // Tracking state.
  Frame::Ptr datum_frame_ = nullptr;  // The datum frame fixed as world frame.
  Frame::Ptr last_frame_ = nullptr;  // Last frame.
  Frame::Ptr curr_frame_ = nullptr;  // Current frame.
  SE3 T_curr_last_;  // Rigid-body transformation from last_frame_ to
                     // curr_frame_ assuming constant velocity.
  // Local covisible keyframes having visual overlapping (i.e. some map points
  // are observed simultaneously by these frames) with current frame.
  unordered_set<Frame::Ptr> local_co_kfs_;
  // FIXME Seems the effect of this is not significant. Remove this?
  int last_kf_id_;  // Id of last keyframe. Frequency of keyframe
                    // insertion is partly(all?) limited by this.

  sptr<Vocabulary> voc_ = nullptr;  // Vocabulary.

  Map::Ptr map_ = nullptr;  // Map.

  Tracking();

  // Entry function.
  void addImage(const cv::Mat& img);

  // Setters.
  void setSystem(sptr<System> system);
  void setLocalMapper(sptr<LocalMapping> local_mapper);
  void setMap(Map::Ptr map);
  void setViewer(sptr<Viewer> viewer);

  void reset();

 private:
  // FIXME Due to errors involved with shared_from_this(), I have to move these
  // two methods from Frame to Tracking.
  // Extract features and compute corresponding descriptors.
  void extractFeatures(const cv::Mat& img);

  // Compute bag of words representation.
  void computeBoW();

  // Track current frame.
  void trackCurrentFrame();

  // Initialize map: collect two consecutive frames and try initialization.
  bool initMap();

  // Track current frame from last frame assuming contant velocity model.
  bool trackFromLastFrame();

  // Track local map to make the tracking more robust.
  bool trackFromLocalMap();

  // Update local covisible keyframes to be used in tracking from local map.
  void updateLocalCoKfs();

  // True if the criteria of inserting a new keyframe are satisfied.
  bool needNewKf();

  // Relocalize if tracking is lost.
  bool relocalization();

 private:
  sptr<System> system_ = nullptr;              // System.
  sptr<LocalMapping> local_mapper_ = nullptr;  // Local mapper.
  sptr<Viewer> viewer_ = nullptr;              // Viewer.

  uptr<Initializer> initializer_ = nullptr;          // Initializer.
  cv::Ptr<cv::FeatureDetector> detector_ = nullptr;  // Feature detector.
};

}  // namespace mono_slam

#endif  // MY_FRONT_END_TRACKING