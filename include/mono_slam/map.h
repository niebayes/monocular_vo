#ifndef MONO_SLAM_MAP_H_
#define MONO_SLAM_MAP_H_

#include "mono_slam/common_include.h"
#include "mono_slam/frame.h"
#include "mono_slam/map_point.h"

namespace mono_slam {

class Frame;
class MapPoint;

// Keyframe database used for relocalization when tracking is lost.
class KeyframeDataBase {
 public:
  using Ptr = sptr<KeyframeDataBase>;

  KeyframeDataBase(sptr<Vocabulary> voc);

  // Add to the lists of keyframes the keyframe sharing words with them.
  void add(Frame::Ptr keyframe);

  // Erase from the lists of keyframes the keyframe sharing words with them.
  void erase(const Frame::Ptr& keyframe);

  // Detect relocalization keyframe candidates between which and the query frame
  // the number of shared words exceed some threshold.
  bool detectRelocCandidates(const Frame::Ptr& frame,
                             list<Frame::Ptr>& candidate_kfs);

  // Clear and reset inverted file indices.
  inline void clear() {
    inv_files_.clear();
    inv_files_.reserve(Config::approx_words_pct() * voc_.size());
  }

 private:
  // Inverted file indices such that inv_files_[i] = list of keyframes having
  // the word with id i (i = 0, 1, ..., length(vocabulary)-1).
  std::unordered_map<int, list<Frame::Ptr>> inv_files_;

  const sptr<Vocabulary> voc_;  // Vocabulary.

  std::mutex mutex_;
};

class Map {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = sptr<Map>;

  Map();

  void insertKeyframe(Frame::Ptr keyframe);

  void eraseKfById(const int id);

  inline int nKfs() const { return static_cast<int>(keyframes_.size()); }

  // FIXME Return copy or const reference?
  inline const list<Frame::Ptr>& getAllKeyframes() {
    u_lock lock(mutex_);
    return keyframes_;
  }

  // FIXME Candidate points?
  inline const list<MapPoint::Ptr>& getAllMapPoints() {
    u_lock lock(mutex_);
    return points_;
  }

  inline void clear() {
    u_lock lock(mutex_);
    keyframes_.clear();
    points_.clear();
    max_keyframe_id_ = 0;
    kf_db_->clear();
  }

  // TODO(bayes) Implement remove functions, e.g. put outlier map points to
  // trash and empty trash properly. And more function like svo.

 private:
  list<Frame::Ptr> keyframes_;  // Maintained keyframes.
  // FIXME Do we really need to maintain map points?
  list<MapPoint::Ptr> points_;  // Maintained map points.

  int max_keyframe_id_;  // Maximum id of keyframes inserted so far. Used for
                         // checking for duplication as new keyframe is comming.

  // Keyframe database used for relocalization.
  KeyframeDataBase::Ptr kf_db_ = nullptr;

  mutable std::mutex mutex_;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_MAP_H_