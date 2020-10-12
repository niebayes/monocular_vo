#include "mono_slam/map.h"

#include "mono_slam/config.h"

namespace mono_slam {

//##############################################################################
// KeyframeDataBase

KeyframeDataBase::KeyframeDataBase(sptr<Vocabulary> voc) : voc_(voc) {
  // Personally, only approximately 20% of ids would be used.
  inv_files_.reserve(Config::approx_n_words_pct() * voc_->size());
}

void KeyframeDataBase::add(Frame::Ptr keyframe) {
  u_lock lock(mut_);
  const DBoW3::BowVector& bow_vec = keyframe->bow_vec_;
  auto it = bow_vec.cbegin(), it_end = bow_vec.cend();
  for (; it != it_end; ++it) inv_files_[it->first].push_back(keyframe);
}

void KeyframeDataBase::erase(const Frame::Ptr& keyframe) {
  lock_g lock(mut_);
  const DBoW3::BowVector& bow_vec = keyframe->bow_vec_;
  auto it = bow_vec.cbegin(), it_end = bow_vec.cend();
  for (; it != it_end; ++it) {
    if (!inv_files_.count(it->first)) continue;
    list<Frame::Ptr>& kfs = inv_files_.at(it->first);
    // FIXME Campturing by reference lose const qualifier?
    std::remove_if(kfs.begin(), kfs.end(), [&keyframe](const Frame::Ptr& kf) {
      return kf == keyframe;
    });
  }
}

bool KeyframeDataBase::detectRelocCandidates(const Frame::Ptr& frame,
                                             list<Frame::Ptr>& candidate_kfs) {
  LOG(INFO) << "Start detecting relocalization candiates ...";
  const steady_clock::time_point t1 = steady_clock::now();

  // Obtain keyframes sharing words with currently quering frame.
  list<Frame::Ptr> kfs_sharing_words;  // Keframes sharing words.
  const DBoW3::BowVector& bow_vec = frame->bow_vec_;
  auto it = bow_vec.cbegin(), it_end = bow_vec.cend();
  {
    lock_g lock(mut_);
    for (; it != it_end; ++it) {
      if (!inv_files_.count(it->first)) continue;
      const list<Frame::Ptr>& kfs = inv_files_.at(it->first);
      for (const Frame::Ptr& kf : kfs) {
        // If not queried yet.
        if (kf->query_frame_id_ != frame->id_) {
          kf->query_frame_id_ = frame->id_;
          kf->n_sharing_words_ = 0;
          kfs_sharing_words.push_back(kf);
        }
        kf->n_sharing_words_++;
      }
    }
  }
  if (kfs_sharing_words.empty()) return false;

  // Find maximal number of sharing words to be used as the indicator to filter
  // out bad keyframe candidates.
  int max_n_sharing_words = 0;
  for (const Frame::Ptr& kf : kfs_sharing_words)
    if (kf->n_sharing_words_ > max_n_sharing_words)
      max_n_sharing_words = kf->n_sharing_words_;
  const int n_sharing_words_thresh = 0.80 * max_n_sharing_words;

  // Filter out bad keyframe candidates and compute bow similarity score.
  list<pair<double, Frame::Ptr>> score_of_kfs;
  for (const Frame::Ptr& kf : kfs_sharing_words) {
    if (kf->n_sharing_words_ <= n_sharing_words_thresh) continue;
    kf->bow_simi_score_ = voc_->score(kf->bow_vec_, frame->bow_vec_);
    score_of_kfs.push_back({kf->bow_simi_score_, kf});
  }

  // Collect covisible keyframes with each candidate keyframe and accumulate
  // their similarity score (only if computed before). The maximal accumulated
  // score is used as the indicator to reject bad covisible keyframe groups.
  list<pair<double, Frame::Ptr>> score_of_best_kfs;
  double max_accu_score = 0;
  for (auto it = score_of_kfs.cbegin(), it_end = score_of_kfs.cend();
       it != it_end; ++it) {
    const Frame::Ptr& kf = it->second;
    // Collect top 10 covisible keyframes ranked wrt. number of shared words.
    const forward_list<Frame::Ptr>& co_kfs = kf->getCoKfs(10);

    // Traverse the covisible keyframes and accumulate the similarity score.
    double max_score_i = kf->bow_simi_score_, accu_score_i = max_score_i;
    Frame::Ptr best_kf_i = kf;
    for (const Frame::Ptr& kf_ : co_kfs) {
      // Only the keyframes visited before are considered having contribution.
      if (kf_->query_frame_id_ != frame->id_) continue;
      if (kf_->bow_simi_score_ > max_score_i) {
        max_score_i = kf_->bow_simi_score_;
        best_kf_i = kf_;
      }
      accu_score_i += kf_->bow_simi_score_;
    }

    // The accumulated score and best keyframe in this group are retained.
    score_of_best_kfs.push_back({accu_score_i, best_kf_i});
    if (accu_score_i > max_accu_score) max_accu_score = accu_score_i;
  }
  const double score_thresh = 0.80 * max_accu_score;

  // Get the candidate keyframes.
  int n_can_kfs = 0;
  for (auto it = score_of_best_kfs.cbegin(), it_end = score_of_best_kfs.cend();
       it != it_end; ++it) {
    if (it->first <= score_thresh || it->second->is_candidate_already_)
      continue;
    candidate_kfs.push_back(it->second);
    it->second->is_candidate_already_ = true;
    ++n_can_kfs;
  }

  const steady_clock::time_point t2 = steady_clock::now();
  const double time_span = duration_cast<duration<double>>(t2 - t1).count();
  LOG(INFO) << n_can_kfs << " relocalization candidates detected.";
  LOG(INFO) << "Relocalization finished in " << time_span << " seconds.";

  // Reset the keyframes.
  std::for_each(score_of_best_kfs.cbegin(), score_of_best_kfs.cend(),
                [](const pair<double, Frame::Ptr>& p) {
                  p.second->is_candidate_already_ = false;
                });
  return true;
}

void KeyframeDataBase::clear() {
  inv_files_.clear();
  inv_files_.reserve(Config::approx_n_words_pct() * voc_->size());
}

//##############################################################################
// Map

Map::Map(sptr<Vocabulary> voc) : voc_(voc), max_kf_id_(0) {
  kf_db_.reset(new KeyframeDataBase(voc_));
}

void Map::insertKeyframe(Frame::Ptr keyframe) {
  lock_g lock(mut_);
  CHECK_EQ(keyframe->isKeyframe(), true);
  if (keyframe->id_ <= max_kf_id_) {
    LOG(WARNING) << "Keyframe being inserted is already in the map.";
    return;
  }
  max_kf_id_ = keyframe->id_;
  kfs_.push_back(keyframe);
  kf_db_->add(keyframe);  // Also add to keyframe database.
  LOG(INFO) << "New keyframe inserted to map.";
}

void Map::insertMapPoint(MapPoint::Ptr point) {
  lock_g lock(mut_);
  points_.push_back(point);
}

void Map::removeKeyframe(const Frame::Ptr& keyframe) {
  lock_g lock(mut_);
  kfs_.remove(keyframe);  // Just call list::remove and that's it!
}

void Map::removeBadMapPoints() {
  lock_g lock(mut_);
  // Just call list::remove_if and that's it!
  points_.remove_if(
      [](const MapPoint::Ptr& point) { return point->to_be_deleted_; });
}

void Map::removeBadObservations(const Frame::Ptr& keyframe,
                                Feature::Ptr& feat) {
  // Avoid repeat removal of map points since a single map point could be
  // observed by many features.
  if (feat->point_.expired()) return;
  {  // Lock since we're changing the state of map points.
    lock_g lock(mut_);
    MapPoint::Ptr point = feat->point_.lock();
    // If less than 3 frames observing this map point after the feature is
    // erased and the observation is marked bad, this map point is removed from
    // map.
    if (point->nObs() - 1 < 3)
      point->to_be_deleted_ = true;
    else {
      // If not goint to be deleted, update infos of the point.
      point->updateBestFeature();
      point->updateMedianViewDirAndScale();
    }
  }
  // Erase observation.
  //! Since map point only "obseres" feature (through weak_ptr) and each feature
  //! is uniquely owned by a keyframe, simply reseat the feature pointer is
  //! sufficient to finish the removal of observation.
  feat.reset();
  // Remove bad map points.
  removeBadMapPoints();  // Remove those marked as to_be_deleted_.
}

void Map::clear() {
  lock_g lock(mut_);
  kfs_.clear();
  max_kf_id_ = 0;
  kf_db_->clear();
}

}  // namespace mono_slam