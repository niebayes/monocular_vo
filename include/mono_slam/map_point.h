#ifndef MONO_SLAM_MAP_POINT_H_
#define MONO_SLAM_MAP_POINT_H_

#include "mono_slam/common_include.h"
#include "mono_slam/feature.h"
#include "mono_slam/frame.h"
#include "mono_slam/g2o_optimizer/g2o_types.h"

namespace mono_slam {

class Frame;
class Feature;

class MapPoint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = sptr<MapPoint>;

  static int point_cnt_;  // Global map point counter, starting from 0.
  const int id_;          // Unique map point identity.
  Vec3 pos_;              // Position in world frame.
  list<sptr<Feature>> observations_;  // List of observations.
  sptr<Feature> best_feat_;  // Best feature in that its descriptor has the
                             // least median distance against other features.
                             // Used for fast matching.

  // These two variables are used in Frame::isObservable. \sa
  // Frame::isObservable.
  Vec3 median_view_dir_;   // Median viewing direction (a unit vector).
  int median_view_scale_;  // Median viewing scale (aka. image pyramid level).

  // Temporary variables used for searching. These variables are updated as
  // Frame::isObservable() is called. \sa Frame::isObservable().
  int curr_tracked_frame_id_;  // Temporary marker storing the id of currently
                               // tracked frame to avoid repeat insertion.
  double repr_x_;  // x coordinate reprojected on currently tracked frame.
  double repr_y_;  // y coordinate reprojected on currently tracked frame.
  int level_;  // Estimated image pyramid level at which searching is performed.
  double cos_view_dir_;  // Cosine of viewing direction from the camera center
                         // of currently tracked frame.

  // Temporary variables used for optimization.
  int curr_ba_keyframe_id_;  // Temporary marker storing the id of currently
                             // bundle-adjusted keyframe to avoid repeat
                             // insertion.

  // Temporary g2o point vertex storing the optimized result.
  sptr<g2o_types::VertexPoint> v_point_ = nullptr;

  bool to_be_deleted_;  // When number of observations below certain threshold,
                        // this map point is going to be deleted soon.

  MapPoint(const Vec3& pos);

  MapPoint(const Vec3& pos, const sptr<Feature>& feat);

  inline const Vec3& pos() const {
    u_lock lock(mutex_);
    return pos_;
  }

  void setPos(const Vec3& pos);

  inline int nObs() const {
    u_lock lock(mutex_);
    //! Since C++11, all(?) STL containers' size() is constant complexity.
    return observations_.size();
  }

  // Add an observation.
  void addObservation(const sptr<Feature>& feat);

  // Erase an observation.
  void eraseObservation(const sptr<Feature>& feat);

  inline list<sptr<Feature>> getObservations() const {
    u_lock lock(mutex_);
    return observations_;
  }

  void updateBestFeature();

  void updateMedianViewDirAndScale();

  // Check if this map point is observed by the given frame.
  // FIXME Incomplete type & forward declaration error if put definition here.
  // FIXME Does inline still work if definition is not here?
  inline bool isObservedBy(const sptr<Frame>& frame) const {
    u_lock lock(mutex_);
    for (const sptr<Feature>& feat : observations_)
      if (feat->frame_.lock() == frame) return true;
    return false;
  }

  // TODO(bayes) Implement delete funtions like svo.

 private:
  mutable std::mutex mutex_;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_MAP_POINT_H_