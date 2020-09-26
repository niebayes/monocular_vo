#include "mono_slam/matcher.h"

namespace mono_slam {
  
Matcher::Matcher(const int matching_threshold,
                 const int distance_ratio_test_threshold)
    : matching_threshold_(matching_threshold),
      distance_ratio_test_threshold_(distance_ratio_test_threshold) {}

int Matcher::SearchForInitialzation(const Frame::Ptr& frame_1,
                                    const Frame::Ptr& frame_2,
                                    vector<int>& matches) {
  const int num_obs_1 = frame_1->NumObs(), num_obs_2 = frame_2->NumObs();
  matches.assign(num_obs_1, -1);
  vector<int> matches_reverse(num_obs_2, -1);

  int num_matches = 0;
  for (int idx_1 = 0; idx_1 < num_obs_1; ++idx_1) {
    const Feature::Ptr& feat_1 = frame_1->feats_[idx_1];
    const int level = feat_1->level;
    if (level > 0) continue;  // Only consider the finest level.
    const vector<int>& feats_indices_2 =
        frame_2->SearchFeatures(feat_1->pt_, 100, level, level);
    if (feats_indices_2.empty()) continue;

    int min_dist = 256, second_min_dist = 256, best_match_idx_2 = -1;
    for (auto it_2 = feats_indices_2.cbegin(),
              it_2_end = feats_indices_2.cend();
         it_2 != it_2_end; ++it_2) {
      const int idx_2 = *it_2;
      const Feature::Ptr& feat_2 = frame_2->feats_[idx_2];
      const int dist =
          ComputeDescriptorDistance(feat_1->descriptor_, feat_2->descriptor_);
      if (dist < min_dist) {
        second_min_dist = min_dist;
        min_dist = dist;
        best_match_idx_2 = idx_2;
      } else if (dist < second_min_dist)
        second_min_dist = dist;
    }

    // Check matching threshold and apply distance ratio test.
    if (dist >= matching_threshold ||
        min_dist >= distance_ratio_test_threshold * second_min_dist)
      continue;
    // Filter out duplicate matches. This further ensures matching quality.
    if (matches_reverse[best_match_idx_2] != -1) {
      matches[matches_reverse[best_match_idx_2]] = -1;
      --num_matches;
    }
    // Update matches.
    matches[idx_1] = best_match_idx_2;
    matches_reverse[best_match_idx_2] = idx_1;
    ++num_matches;
  }
  return num_matches;
}

namespace matcher_utils {

int ComputeDescriptorDistance(const cv::Mat& desc_1, const cv::Mat& desc_2) {
  const int* pa = a.ptr<int32_t>();
  const int* pb = b.ptr<int32_t>();

  int dist = 0;

  for (int i = 0; i < 8; i++, pa++, pb++) {
    unsigned int v = *pa ^ *pb;
    v = v - ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }

  return dist;
}
}  // namespace matcher_utils
}  // namespace mono_slam
