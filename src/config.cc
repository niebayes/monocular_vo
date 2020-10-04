#include "mono_slam/config.h"

namespace mono_slam {

Config::Config()
    : max_n_feats_(),
      min_n_feats_(),
      min_n_matches_(),
      min_n_inlier_matches_(),
      init_min_n_feats_(),
      init_min_n_matches_(),
      init_min_n_inlier_matches_(),
      reloc_n_iters_p3p_(5),
      reloc_min_n_matches_(),
      reloc_min_n_inlier_matches_(),
      tri_min_n_matches_(),
      tri_min_parallax_(1.0),
      match_thresh_relax_(100),
      match_thresh_strict_(50),
      search_radius_(),
      search_view_dir_factor_low_(2.5),
      search_view_dir_factor_high_(4.0),
      scale_factor_(1.2),
      scale_factors_(),
      scale_n_levels_(8),
      scale_level_sigma2_(),
      dist_ratio_test_factor_(0.8),
      redun_factor_(0.9),
      weight_factor_(),
      new_kf_interval_(),
      max_n_kfs_in_map_(),
      approx_n_words_pct_(0.2),
      co_kf_weight_thresh_() {}

Config& Config::getInstance() {
  // Instantiated on first use and guaranteed to be destroyed
  static Config instance;
  return instance;
}

}  // namespace mono_slam