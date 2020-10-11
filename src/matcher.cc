#include "mono_slam/matcher.h"

#include "DBoW3/DBoW3.h"
#include "mono_slam/config.h"
#include "mono_slam/feature.h"
#include "mono_slam/geometry_solver.h"

namespace mono_slam {

int Matcher::searchForInitialization(const Frame::Ptr& ref_frame,
                                     const Frame::Ptr& curr_frame,
                                     vector<int>& matches) {
  const int n_obs_1 = ref_frame->nObs(), n_obs_2 = curr_frame->nObs();
  matches.assign(n_obs_1, -1);  // -1 denotes no matching.
  // Record as well reverse matching to avoid repeat matching.
  vector<bool> matched(n_obs_2, false);

  int n_matches = 0;
  for (int idx_1 = 0; idx_1 < n_obs_1; ++idx_1) {
    const Feature::Ptr& feat_1 = ref_frame->feats_[idx_1];
    const int level = feat_1->level_;
    if (level > 0) continue;  // Only consider the finest level.
    const vector<int> feat_indices_2 = curr_frame->searchFeatures(
        feat_1->pt_, Config::search_radius(), level, level);
    if (feat_indices_2.empty()) continue;

    int min_dist = 256, second_min_dist = 256, best_idx_2 = 0;
    for (const int idx_2 : feat_indices_2) {
      if (matched[idx_2]) continue;  // Avoid repeat matching.
      const Feature::Ptr& feat_2 = curr_frame->feats_[idx_2];
      const int dist = matcher_utils::computeDescDist(feat_1->descriptor_,
                                                      feat_2->descriptor_);
      if (dist < min_dist) {
        second_min_dist = min_dist;
        min_dist = dist;
        best_idx_2 = idx_2;
      } else if (dist < second_min_dist)
        second_min_dist = dist;
    }

    // Check matching threshold and apply distance ratio test.
    if (min_dist >= Config::match_thresh_relax() ||
        min_dist >= Config::dist_ratio_test_factor() * second_min_dist)
      continue;
    // Update matches.
    matches[idx_1] = best_idx_2;
    matched[best_idx_2] = true;
    ++n_matches;
  }
  return n_matches;
}

int Matcher::searchByProjection(const Frame::Ptr& last_frame,
                                const Frame::Ptr& curr_frame) {
  return Matcher::searchByProjection(unordered_set<Frame::Ptr>{last_frame},
                                     curr_frame);
}

int Matcher::searchByProjection(const unordered_set<Frame::Ptr>& local_co_kfs,
                                const Frame::Ptr& curr_frame) {
  if (local_co_kfs.empty()) return 0;
  int n_matches = 0;

  // Iterate each keyframe->feature->map_point to find the best matches between
  // the map_point and features in curr_frame.

  // Helper container used to reset the marker.
  unordered_set<MapPoint::Ptr> shared_points;
  for (const Frame::Ptr& kf : local_co_kfs) {
    for (const Feature::Ptr& feat : kf->feats_) {
      const MapPoint::Ptr& point = feat_utils::getPoint(feat);
      if (!point || point->curr_tracked_frame_id_ == curr_frame->id_) continue;
      point->curr_tracked_frame_id_ = curr_frame->id_;
      shared_points.insert(point);
      if (!curr_frame->isObservable(point, feat->level_)) continue;

      // Perform 3D-2D searching.
      // Search radius is enlarged at larger scale and also influenced by
      // viewing direction from the camera center of current frame.
      const int level = point->level_;
      const int search_radius =
          Config::search_radius() *
          Config::search_view_dir_factor(point->cos_view_dir_) *
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
        const Feature::Ptr& feat_i = curr_frame->feats_[idx];
        // Only consider unmatched features.
        if (!feat_i->point_.expired()) continue;
        const int dist = matcher_utils::computeDescDist(
            point->best_feat_->descriptor_, feat_i->descriptor_);
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
      if (min_dist >= Config::match_thresh_relax() ||
          min_dist >= Config::dist_ratio_test_factor() * second_min_dist)
        // ||
        // best_level != second_best_level)
        continue;

      // Update linked map point.
      // FIXME Does weak_ptr incurs issues here?
      //! Currently the point is associated with the feature and the frame but
      //! the observation information of the point is not updated yet. (It will
      //! be updated by the local mapper).
      curr_frame->feats_[best_idx]->point_ = point;
      ++n_matches;
    }
  }
  // Reset the marker making it ready for the next searching.
  std::for_each(
      shared_points.cbegin(), shared_points.cend(),
      [](const MapPoint::Ptr& point) { point->curr_tracked_frame_id_ = -1; });
  return n_matches;
}

int Matcher::searchByBoW(const Frame::Ptr& keyframe, const Frame::Ptr& frame,
                         vector<int>& matches) {
  const vector<Feature::Ptr>& feats_kf = keyframe->feats_;
  const vector<Feature::Ptr>& feats_f = frame->feats_;
  const int n_feats_kf = feats_kf.size(), n_feats_f = feats_f.size();
  matches.assign(n_feats_kf, -1);  // -1 denotes no matching.
  // Record as well reverse matches to preclude repeat matching.
  vector<bool> matched(n_feats_f, false);

  int n_matches = 0;
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
          if (matched[idx_f]) continue;  // Avoid repeat matching.
          const int dist = matcher_utils::computeDescDist(
              feats_kf[idx_kf]->descriptor_, feats_f[idx_f]->descriptor_);
          if (dist < min_dist) {
            second_min_dist = dist;
            min_dist = dist;
            best_idx_f = idx_f;
          } else if (dist < second_min_dist)
            second_min_dist = dist;
        }

        // Apply thresholding test and distance ratio test.
        if (min_dist >= Config::match_thresh_strict() ||
            min_dist >= Config::dist_ratio_test_factor() * second_min_dist)
          continue;
        matches[idx_kf] = best_idx_f;
        matched[best_idx_f] = true;
        ++n_matches;
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
  return n_matches;
}

int Matcher::searchForTriangulation(const Frame::Ptr& keyframe_1,
                                    const Frame::Ptr& keyframe_2,
                                    vector<int>& matches) {
  const vector<Feature::Ptr>& feats_1 = keyframe_1->feats_;
  const vector<Feature::Ptr>& feats_2 = keyframe_2->feats_;
  const int n_feats_1 = feats_1.size(), n_feats_2 = feats_2.size();
  matches.assign(n_feats_1, -1);  // -1 denotes no matching.
  // Record as well reverse matches to preclude repeat matching.
  vector<bool> matched(n_feats_2, false);

  int n_matches = 0;
  // Searching feature matches by utilizing feature vectors formed by vocabulary
  // tree.
  auto it_1 = keyframe_1->feat_vec_.cbegin(),
       it_1_end = keyframe_1->feat_vec_.cend(),
       it_2 = keyframe_2->feat_vec_.cbegin(),
       it_2_end = keyframe_2->feat_vec_.cend();
  // Perform searching till exhausted.
  while (it_1 != it_1_end && it_2 != it_2_end) {
    // Search feature matches in the same node.
    if (it_1->first == it_2->first) {
      // Each node in the vocabulary contains indices of feature descriptors of
      // the correponding features detected in the frame.
      const vector<unsigned int>& indices_1 = it_1->second;
      const vector<unsigned int>& indices_2 = it_2->second;

      for (const int idx_1 : indices_1) {
        const Feature::Ptr& feat_1 = feats_1[idx_1];
        // Only consider unmatched features.
        if (!feat_1->point_.expired()) continue;

        // Search feature matches between the feature in keyframe_1 and all
        // features in keyframe_2.
        int min_dist = 256, second_min_dist = 256;
        int best_idx_2 = 0;
        for (const int idx_2 : indices_2) {
          const Feature::Ptr& feat_2 = feats_2[idx_2];
          // Skip those features that already link a map point or have matched
          // before.
          if (!feat_2->point_.expired() || matched[idx_2]) continue;

          // Compute descriptor distance.
          const int dist = matcher_utils::computeDescDist(feat_1->descriptor_,
                                                          feat_2->descriptor_);
          if (dist < min_dist) {
            second_min_dist = dist;
            min_dist = dist;
            best_idx_2 = idx_2;
          } else if (dist < second_min_dist)
            second_min_dist = dist;
        }

        // Apply thresholding test, distance ratio test and epipolar constraint
        // test.
        if (min_dist >= Config::match_thresh_strict() ||
            min_dist >= Config::dist_ratio_test_factor() * second_min_dist)
          continue;
        const Mat33 F_2_1 =
            geometry::getFundamentalByPose(keyframe_1, keyframe_2);
        const Feature::Ptr& feat_2 = feats_2[best_idx_2];
        const double dist_1 = geometry::pointToEpiLineDist(
                         feat_1->pt_, feat_2->pt_, F_2_1, true),
                     dist_2 = geometry::pointToEpiLineDist(
                         feat_1->pt_, feat_2->pt_, F_2_1, false);
        const double chi2_thresh = 3.84;  // One degree chi-square p-value;
        const vector<double>& sigma2s = Config::scale_level_sigma2();
        if (dist_1 >= chi2_thresh * sigma2s.at(feat_1->level_) ||
            dist_2 >= chi2_thresh * sigma2s.at(feat_2->level_))
          continue;
        matches[idx_1] = best_idx_2;
        matched[best_idx_2] = true;
        ++n_matches;
      }
      ++it_1;
      ++it_2;
    } else if (it_1->first < it_2->first) {
      // Align the iterators of keyframe_1 with that of keyframe_2.
      it_1 = keyframe_2->feat_vec_.lower_bound(it_2->first);
    } else {
      // Align the iterators of keyframe_2 with that of keyframe_1
      it_2 = keyframe_1->feat_vec_.lower_bound(it_1->first);
    }
  }
  return n_matches;
}

namespace matcher_utils {

int computeDescDist(const cv::Mat& desc_1, const cv::Mat& desc_2) {
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
