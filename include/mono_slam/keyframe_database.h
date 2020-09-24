#ifndef MONO_SLAM_KEYFRAME_DATABASE_H_
#define MONO_SLAM_KEYFRAME_DATABASE_H_

#include "mono_slam/common_include.h"

namespace mono_slam {

class KeyframeDB {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = sptr<KeyframeDB>;

  KeyframeDB(const sptr<Vocabulary>& voc);

  void AddKeyframe(const Frame::Ptr& keyframe);

  void EraseKeyframe(const Frame::Ptr& keyframe);

  void Reset();

 private:
  // Vocabulary.
  const sptr<Vocabulary> voc_;

  // Inverted indices.
  vector<list<Frame::Ptr>> inverted_indices_;
};

KeyframeDB::KeyframeDB(const sptr<Vocabulary>& voc) : voc_(voc) {}

}  // namespace mono_slam

#endif  // MONO_SLAM_KEYFRAME_DATABASE_H_