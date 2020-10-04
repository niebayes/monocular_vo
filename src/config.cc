#include "mono_slam/config.h"

namespace mono_slam {

Config::Config()
    : approx_n_words_pct_(0.2),
      co_kf_weight_thresh_(),
      reloc_n_iters_p3p_(5),
      tri_min_n_matches_(),
      tri_min_parallax_(1.0),
      scale_factors_(),
      scale_level_sigma2_(),
      redun_factor_(0.9),
      match_thresh_relax_(100),
      match_thresh_strict_(50),
      dist_ratio_test_factor_(0.8),
      search_radius_(),
      search_view_dir_factor_low_(2.5),
      search_view_dir_factor_high_(4.0) {}

}  // namespace mono_slam