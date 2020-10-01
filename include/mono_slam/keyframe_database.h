#ifndef MONO_SLAM_KEYFRAME_DATABASE_H_
#define MONO_SLAM_KEYFRAME_DATABASE_H_

#include "mono_slam/common_include.h"
#include "mono_slam/frame.h"

namespace mono_slam {

class Frame;

class KeyframeDB {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = sptr<KeyframeDB>;

  KeyframeDB(const sptr<Vocabulary>& voc);

  void addKeyframe(const Frame::Ptr& keyframe);

  void eraseKeyframe(const Frame::Ptr& keyframe);

  void reset();

 private:
  // Vocabulary.
  const sptr<Vocabulary> voc_;

  // Inverted indices.
  vector<list<Frame::Ptr>> inverted_indices_;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_KEYFRAME_DATABASE_H_