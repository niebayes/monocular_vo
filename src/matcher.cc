#include "mono_slam/matcher.h"

#include "mono_slam/feature.h"

namespace mono_slam {

int Matcher::searchForInitialization(const Frame::Ptr& ref_frame,
                                     const Frame::Ptr& curr_frame,
                                     vector<int>& matches) {
  const int num_obs_1 = ref_frame->nObs(), num_obs_2 = curr_frame->nObs();
  matches.assign(num_obs_1, -1);
  vector<int> matches_reverse(num_obs_2, -1);

  int num_matches = 0;
  for (int idx_1 = 0; idx_1 < num_obs_1; ++idx_1) {
    const Feature::Ptr& feat_1 = ref_frame->feats_[idx_1];
    const int level = feat_1->level_;
    if (level > 0) continue;  // Only consider the finest level.
    const vector<int> feat_indices_2 =
        curr_frame->searchFeatures(feat_1->pt_, 100, level, level);
    if (feat_indices_2.empty()) continue;

    int min_dist = 256, second_min_dist = 256, best_match_idx_2 = -1;
    for (int idx_2 : feat_indices_2) {
      const Feature::Ptr& feat_2 = curr_frame->feats_[idx_2];
      const int dist = matcher_utils::computeDescriptorDistance(
          feat_1->descriptor_, feat_2->descriptor_);
      if (dist < min_dist) {
        second_min_dist = min_dist;
        min_dist = dist;
        best_match_idx_2 = idx_2;
      } else if (dist < second_min_dist)
        second_min_dist = dist;
    }

    // Check matching threshold and apply distance ratio test.
    if (min_dist >= Config::matching_thresh_relax() ||
        min_dist >= Config::dist_ratio_test_factor() * second_min_dist)
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

int Matcher::searchByProjection(const Frame::Ptr& last_frame,
                                const Frame::Ptr& curr_frame) {
  return Matcher::searchByProjection(std::set{last_frame}, curr_frame);
}

int Matcher::searchByProjection(const std::set<Frame::Ptr>& local_co_kfs,
                                const Frame::Ptr& curr_frame) {
  if (local_co_kfs.empty()) return 0;
  int num_matches = 0;

  // Iterate each keyframe->feature->map_point to find best match between the
  // map_point and features in curr_frame.
  for (const auto& keyframe : local_co_kfs) {
    for (const auto& feat_ : keyframe->feats_) {
      if (feat_.expired()) continue;
      const auto& feat = feat_.lock();
      if (feat->point_.expire()) continue;
      const auto& point = feat->point_.lock();
      if (point->curr_tracked_frame_id_ == curr_frame->id_ ||
          point->to_be_deleted_)
        continue;
      point->curr_tracked_frame_id_ = curr_frame->id_;
      if (!curr_frame->isObservable(point)) continue;

      // Perform 3D-2D searching.
      // Search radius is enlarged at larger scale and also influenced by
      // viewing direction from the camera center of current frame.
      const int level = point->level_;
      const int search_radius = Config::search_radius() *
                                Config::search_factor(point->cos_view_dir_) *
                                Config::scale_factors().at(level);
      const vector<int> feat_indices =
          curr_frame->searchFeatures(Vec2{point->repr_x_, point->repr_y_},
                                     search_radius, level - 1, level + 1);
      if (feat_indices.empty()) continue;

      // Iterate all matched features in current frame to find best and second
      // best matches.
      int min_dist = 256, second_min_dist = 256;
      int best_level = 0, second_best_level = 0;
      int best_idx = 0;
      for (int idx : feat_indices) {
        // FIXME Should I always check expired for weak_ptr before lock?
        const auto& feat_i = curr_frame->feats_.at(idx).lock();
        // Only consider unmatched features.
        if (!feat_i->point_.expired()) continue;
        const int dist = matcher_utils::computeDescDist(
            point->best_feat_.lock()->descriptor_, feat_i->descriptor_);
        if (dist < min_dist) {
          second_min_dist = min_dist;
          min_dist = dist;
          second_best_level = best_level;
          best_level = feat_i->level_;
          best_idx = idx;
        } else if (dist < second_min_dist) {
          second_min_dist = dist;
          second_best_level = feat_i->level_;
        }
      }

      // Perform thresholding, distance ratio test, and scale consistency test,
      if (min_dist >= Config::matching_thresh_relax() ||
          min_dist >= Config::dist_ratio_test_factor() * second_min_dist ||
          best_level != second_best_level)
        continue;

      // Update linked map point.
      // FIXME Does weak_ptr incurs issues here?
      curr_frame->feats_.at(best_idx).lock()->point_ = point;
      ++num_matches;
    }
  }
  return num_matches;
}

int Matcher::searchByBoW(const Frame::Ptr& keyframe, const Frame::Ptr& frame,
                         vector<int>& matches) {
  const vector<Feature::Ptr&> feats_kf = keyframe->feats_;
  const vector<Feature::Ptr&> feats_f = frame->feats_;
  const int num_feats_kf = feats_kf.size(), num_feats_f = feats_f.size();
  matches.assign(num_feats_kf, -1);  // -1 denotes no matching.
  // Record as well reverse matches to preclude repeat matching.
  vector<int> matches_reverse(num_feats_f, -1);

  int num_matches = 0;
  // Searching feature matches by utilizing feature vectors formed by vocabulary
  // tree.
  auto it_kf = keyframe->feat_vec_.cbegin(),
       it_kf_end = keyframe->feat_vec_.cend(), it_f = frame->feat_vec_.cbegin(),
       it_f_end = frame->feat_vec_.cend();
  // Perform searching till exhausted.
  while (it_kf != it_kf_end && it_f != it_f_end) {
    // Search feature matches in the same node.
    if (it_kf->first == it_f->first) {
      // Each node in the vocabulary contains indices of feature descriptors of
      // the correponding features detected in the frame.
      const vector<unsigned int>& indices_kf = it_kf->second;
      const vector<unsigned int>& indices_f = it_f->second;

      for (const int idx_kf : indices_kf) {
        const Feature::Ptr& feat_kf = feats_kf[idx_kf];
        const MapPoint::Ptr& point = feat_utils::getPoint(feat_kf);
        // Since we're searching for 3D-2D matches, the corresponding point of
        // this feature must be valid.
        if (!point) continue;

        // Search feature matches between the feature in keyframe and all
        // features in frame.
        int min_dist = 256, second_min_dist = 256;
        int best_idx_f = 0;
        for (const int idx_f : indices_f) {
          if (matches_reverse[idx_f] != -1) continue;  // Avoid repeat matching.
          const int dist = matched_utils::computeDescDist(
              feats_kf[idx_kf]->descriptor_, feats_f[idx_f]->descriptor_);
          if (dist < min_dist) {
            second_min_dist = dist;
            min_dist = dist;
            best_idx_f = idx_f;
          } else if (dist < second_min_dist)
            second_min_dist = dist;
        }

        // Apply thresholding test and distance ratio test.
        if (min_dist >= Config::matching_thresh_relax() ||
            min_dist >= Config::dist_ratio_test_factor() * second_min_dist)
          continue;
        matches[idx_kf] = best_idx_f;
        matches_reverse[best_idx_f] = idx_kf;
        ++num_matches;
      }
      ++it_kf;
      ++it_f;
    } else if (it_kf->first < it_f->first) {
      // Align the iterators of keyframe with that of frame.
      it_kf = frame->feat_vec_.lower_bound(it_f->first);
    } else {
      // Align the iterators of frame with that of keyframe.
      it_f = keyframe->feat_vec_.lower_bound(it_kf->first);
    }
  }
  return num_matches;
}

namespace matcher_utils {

static inline int computeDescDist(const cv::Mat& desc_1,
                                  const cv::Mat& desc_2) {
  const int* pa = desc_1.ptr<int32_t>();
  const int* pb = desc_2.ptr<int32_t>();

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
