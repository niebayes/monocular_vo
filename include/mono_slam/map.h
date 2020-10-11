#ifndef MONO_SLAM_MAP_H_
#define MONO_SLAM_MAP_H_

#include "DBoW3/DBoW3.h"
#include "mono_slam/common_include.h"
#include "mono_slam/config.h"
#include "mono_slam/frame.h"
#include "mono_slam/map_point.h"

using DBoW3::Vocabulary;

namespace mono_slam {

class Frame;

// Keyframe database used for relocalization when tracking is lost.
class KeyframeDataBase {
 public:
  using Ptr = uptr<KeyframeDataBase>;

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
  void clear();

 private:
  // Inverted file indices such that inv_files_[i] = list of keyframes having
  // the word with id i (i = 0, 1, ..., length(vocabulary)-1).
  unordered_map<int, list<Frame::Ptr>> inv_files_;

  const sptr<Vocabulary> voc_{nullptr};  // Vocabulary.

  std::mutex mut_;
};

struct Feature;
class MapPoint;

class Map {
 public:
  using Ptr = sptr<Map>;
  // Keyframe database used for relocalization.
  KeyframeDataBase::Ptr kf_db_{nullptr};

  Map(sptr<Vocabulary> voc);

  void insertKeyframe(Frame::Ptr keyframe);

  void insertMapPoint(MapPoint::Ptr point);

  // TODO(bayes) Implement remove functions, e.g. put outlier map points to
  // trash and empty trash properly. And more function like svo.
  void removeBadObservation(const Frame::Ptr& keyframe, Feature::Ptr& feat);

  void eraseKfById(const int id);

  inline int nKfs() const {
    lock_g lock(mut_);
    return static_cast<int>(kfs_.size());
  }

  // FIXME Return copy or const reference?
  inline const list<Frame::Ptr>& getAllKeyframes() const {
    lock_g lock(mut_);
    return kfs_;
  }

  inline const list<MapPoint::Ptr>& getAllMapPoints() const {
    lock_g lock(mut_);
    return points_;
  }

  void clear();

 private:
  list<Frame::Ptr> kfs_;        // Maintained keyframes.
  list<MapPoint::Ptr> points_;  // Maintained map points;
  int max_kf_id_;  // Maximum id of keyframes inserted so far. Used for
                   // checking for duplication as new keyframe is comming.
  sptr<Vocabulary> voc_{nullptr};

  mutable std::mutex mut_;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_MAP_H_