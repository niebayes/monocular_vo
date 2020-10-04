#ifndef MONO_SLAM_CONFIG_H_
#define MONO_SLAM_CONFIG_H_

#include "mono_slam/common_include.h"

namespace mono_slam {

//! Global configuration class. Other user-provided configurations, e.g. file
//! paths, camera parameters, etc. will be passed in through YAML file.

// Implement the Singleton design pattern to allow global access and to ensure
// that only one instance exists.
class Config {
 public:
  Config& getInstance();

  // Maximum number of features to be detected when a new frame is created.
  static int& max_n_feats() { return getInstance().max_n_feats_; }

  static int& min_n_feats() { return getInstance().min_n_feats_; }

  static int& min_n_matches() { return getInstance().min_n_matches_; }

  static int& min_n_inlier_matches() {
    return getInstance().min_n_inlier_matches_;
  }

  // Minimun number of features needed to do map initialization.
  static int& init_min_n_feats() { return getInstance().init_min_n_feats_; }

  // Minimum number of matches needed to do map initialization.
  static int& init_min_n_matches() { return getInstance().init_min_n_matches_; }

  // Minimum number of inlier matches survived from global bundle adjustment
  // during map initialization.
  static int& init_min_n_inlier_matches() {
    return getInstance().init_min_n_inlier_matches_;
  }

  // Number of iterations P3P solving is performed during relocalization.
  static int& reloc_n_iters_p3p() { return getInstance().reloc_n_iters_p3p_; }

  static int& reloc_min_n_matches() {
    return getInstance().reloc_min_n_matches_;
  }

  static int& reloc_min_n_inlier_matches() {
    return getInstance().reloc_min_n_inlier_matches_;
  }

  // Minimum number of matches below which the triangulation is rejected.
  static int& tri_min_n_matches() { return getInstance().tri_min_n_matches_; }

  // Minimum parallax below which the triangulation is rejected.
  static int& tri_min_parallax() { return getInstance().tri_min_parallax_; }

  // Relax matching threshold used cases other than searching driven by bag of
  // words.
  static int& match_thresh_relax() { return getInstance().match_thresh_relax_; }

  // Strict matching threshold used in searchinng driven by bag of words.
  static int& match_thresh_strict() {
    return getInstance().match_thresh_strict_;
  }

  static int& search_radius() { return getInstance().search_radius_; }

  // Searching factor of differene viewing direction.
  static double& search_view_dir_factor(const double cos_view_dir) {
    if (cos_view_dir > 0.998)  // When viewing direction less than 3.6 degree.
      return getInstance().search_view_dir_factor_low_;
    else  // When viewing direction greater than 3.6 degree.
      return getInstance().search_view_dir_factor_high_;
  }

  // Scale factor used for creating image pyramid during feature extraction.
  static double& scale_factor() { return getInstance().scale_factor_; }

  // Scale factors of each image pyramid level.
  static vector<double>& scale_factors() {
    return getInstance().scale_factors_;
  }

  // Number of image pyramid levels.
  static int& scale_n_levels() { return getInstance().scale_n_levels_; }

  // Squared noise sigmas of each image pyramid level.
  static vector<double>& scale_level_sigma2() {
    return getInstance().scale_level_sigma2_;
  }

  // The ratio of minimum distance and secondminimum distance below which the
  // match is rejected.
  static double& dist_ratio_test_factor() {
    return getInstance().dist_ratio_test_factor_;
  }

  // The ratio of number of redundant map points and number of valid map points
  // below which the keyframe is marked as redundant.
  static double& redun_factor() { return getInstance().redun_factor_; }

  static double& weight_factor() { return getInstance().weight_factor_; }

  static int& new_kf_interval() { return getInstance().new_kf_interval_; }

  static int& max_n_kfs_in_map() { return getInstance().max_n_kfs_in_map_; }

  // Approximated percentage of number of words will be used in current scene.
  static double& approx_n_words_pct() {
    return getInstance().approx_n_words_pct_;
  }

  // Number of shared map poinnts below which the connection between two
  // keyframes will not be seen valid.
  static int& co_kf_weight_thresh() {
    return getInstance().co_kf_weight_thresh_;
  }

 private:
  // Private constructor preventing instantiation to make a singleton (i.e. no
  // objects can be created).
  Config();

  // Global Configurations.
  int max_n_feats_;
  int min_n_feats_;
  int min_n_matches_;
  int min_n_inlier_matches_;
  int init_min_n_feats_;
  int init_min_n_matches_;
  int init_min_n_inlier_matches_;
  int reloc_n_iters_p3p_;
  int reloc_min_n_matches_;
  int reloc_min_n_inlier_matches_;
  int tri_min_n_matches_;
  double tri_min_parallax_;
  int match_thresh_relax_;
  int match_thresh_strict_;
  int search_radius_;
  double search_view_dir_factor_low_;
  double search_view_dir_factor_high_;
  double scale_factor_;
  vector<double> scale_factors_;
  int scale_n_levels_;
  vector<double> scale_level_sigma2_;
  double dist_ratio_test_factor_;
  double redun_factor_;
  double weight_factor_;
  int new_kf_interval_;
  int max_n_kfs_in_map_;
  double approx_n_words_pct_;
  int co_kf_weight_thresh_;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_CONFIG_H_
