#include "mono_slam/map.h"

#include "mono_slam/config.h"

namespace mono_slam {

//##############################################################################
// KeyframeDataBase

KeyframeDataBase::KeyframeDataBase(sptr<Vocabulary> voc) : voc_(voc) {
  // Personally, only approximately 20% of ids would be used.
  inv_files_.reserve(Config::approx_words_pct() * voc_.size());
}

void KeyframeDataBase::add(Frame::Ptr keyframe) {
  CHECK_EQ(keyframe->isKeyframe(), true);
  u_lock lock(mutex_);
  const DBoW3::BowVector& bow_vec = keyframe->bow_vec_;
  auto it = bow_vec.cbegin(), it_end = bow_vec.cend();
  for (; it != it_end; ++it) inv_files_[it->first].push_back(keyframe);
}

void KeyframeDataBase::erase(const Frame::Ptr& keyframe) {
  CHECK_EQ(keyframe->isKeyframe(), false);
  u_lock lock(mutex_);
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
  // FIXME Is this okay not putting parenthese critical section?
  u_lock lock(mutex_);

  // Obtain keyframes sharing words with currently quering frame.
  list<Frame::Ptr> kfs_sharing_words;  // Keframes sharing words.
  const DBoW3::BowVector& bow_vec = keyframe->bow_vec_;
  auto it = bow_vec.cbegin(), it_end = bow_vec.cend();
  for (; it != it_end; ++it) {
    if (!inv_files_.count(it->first)) continue;
    list<Frame::Ptr>& kfs = inv_files_.at(it->first);
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
    kf_->simi_score = voc_->score(kf->bow_vec_, frame->bow_vec_);
    score_of_kfs.push_back({kf_->simi_score, kf});
  }

  // Collect covisible keyframes with each candidate keyframe and accumulate
  // their similarity score (only if computed before). The maximal accumulated
  // score is used as the indicator to reject bad covisible keyframe groups.
  list<pair<double, Frame::Ptr>> score_of_best_kfs;
  double max_accu_score = 0;
  for (auto it = score_of_kfs.cbegin(), it_end = score_of_kfs.cend(); ++it) {
    const Frame::Ptr& kf = it->second;
    // Collect top 10 covisible keyframes ranked wrt. number of shared words.
    const set<Frame::Ptr>& co_kfs = kf->getCovisibleKeyframes(10);

    // Traverse the covisible keyframes and accumulate the similarity score.
    double max_score_i = kf->simi_score_, accu_score_i = max_score_i;
    Frame::Ptr best_kf_i = nullptr;
    for (const Frame::Ptr& kf_ : co_kfs) {
      // Only the keyframes visited before are considered having contribution.
      if (kf_->query_frame_id_ != frame->id_) continue;
      if (kf_->simi_score_ > max_score_i) {
        max_score_i = kf_->simi_score_;
        best_kf_i = kf_;
      }
      accu_score_i += kf_->simi_score_;
    }

    // The accumulated score and best keyframe in this group are retained.
    score_of_best_kfs.push_back({accu_score_i, best_kf_i});
    if (accu_score_i > max_accu_score) max_accu_score = accu_score_i;
  }
  const double score_thresh = 0.80 * max_accu_score;

  // Get the candidate keyframes.
  for (auto it = score_of_best_kfs.cbegin(), it_end = score_of_best_kfs.cend();
       it != it_end; ++it) {
    if (it.first <= score_thresh || it->second->is_candidate_already_) continue;
    candidate_kfs.push_back(it->second);
    it->second->is_candidate_already_ = true;
  }
  // Reset the keyframes.
  std::for_each(
      score_of_best_kfs.cbegin(), score_of_best_kfs.cend(),
      [](const Frame::Ptr& kf) { kf->is_candidate_already_ = false; });
}

//##############################################################################
// Map

void Map::ensertKeyframe(Frame::Ptr keyframe) {
  CHECK_EQ(keyframe->isKeyframe(), true);
  CHECK_GE(keyframe->id_, max_frame_id_);
  u_lock lock(mutex_);
  keyframes_.push_back(keyframe);
}

// FIXME What for?
void Map::eraseKeyframeById(const int id) {
  u_lock lock(mutex_);
  // FIXME Efficiency issue? Use for loop and break out once found?
  keyframes_.remove_if(
      [](const Frame::Ptr& keyframe) { return keyframe->id_ == id; });
}

}  // namespace mono_slam