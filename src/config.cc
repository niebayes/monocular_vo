#include "mono_slam/config.h"

namespace mono_slam {

Config::Config()
    : max_n_feats_(2000),
      min_n_feats_(100),
      min_n_matches_(10),
      min_n_inlier_matches_(10),
      init_min_n_feats_(200),
      init_min_n_matches_(70),
      init_min_n_inlier_matches_(50),
      init_min_n_triangulated_(40),
      reloc_n_iters_p3p_(5),
      reloc_min_n_matches_(25),
      reloc_min_n_inlier_matches_(20),
      tri_min_n_matches_(50),
      tri_min_parallax_(1.0),
      match_thresh_relax_(100),
      match_thresh_strict_(50),
      search_radius_(100),
      search_view_dir_factor_low_(2.5),
      search_view_dir_factor_high_(4.0),
      scale_factor_(1.2),
      scale_n_levels_(8),
      dist_ratio_test_factor_(0.8),
      redun_factor_(0.9),
      weight_factor_(0.8),
      new_kf_interval_(5),
      max_n_kfs_in_map_(50),
      approx_n_words_pct_(0.2),
      co_kf_weight_thresh_(15) {
  // Generate scale factors for each image pyramid level.
  scale_factors_.resize(scale_n_levels_);
  std::iota(scale_factors_.begin(), scale_factors_.end(), 0);
  std::transform(scale_factors_.begin(), scale_factors_.end(),
                 scale_factors_.begin(),
                 [=](double i) { return std::pow(scale_factor_, i); });

  // Compute squared noise sigmas for each image pyramid level.
  scale_level_sigma2_.resize(scale_n_levels_);
  std::transform(scale_factors_.cbegin(), scale_factors_.cend(),
                 scale_level_sigma2_.begin(),
                 [](const double i) { return i * i; });
}

Config& Config::getInstance() {
  // Instantiated on first use and guaranteed to be destroyed
  static Config instance;
  return instance;
}

}  // namespace mono_slam