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

 private:
  // Private constructor preventing instantiation to make a singleton (i.e. no
  // objects can be created).
  Config();

  // Global Configurations.
  double approx_n_words_pct_;  // Approximated percentage of number of words
                               // will be used in current scene.
  int co_kf_weight_thresh_;    // Number of shared map poinnts below which the
                               // connection between two keyframes will not be
                               // seen valid.
  int reloc_n_iters_p3p_;      // Number of iterations P3P solving is performed
                               // during relocalization.
  int reloc_min_n_matches_;
  int reloc_min_n_inlier_matches_;
  int tri_min_n_matches_;  // Minimum number of matches below which the
                           // triangulation is rejected.
  int tri_min_parallax_;   // Minimum parallax below which the triangulation is
                           // rejected.
  double scale_factor_;
  vector<double> scale_factors_;  // Scale factors of each image pyramid level.
  int n_scale_levels_;
  vector<double>
      scale_level_sigma2_;  // Squared noise sigmas of each image pyramid level.
  double redun_factor_;     // The ratio of number of redundant map points and
                         // number of valid map points below which the keyframe
                         // is marked as redundant.
  int match_thresh_relax_;   // Relax matching threshold used cases other than
                             // searching driven by bag of words.
  int match_thresh_strict_;  // Strict matching threshold used in searchinng
                             // driven by bag of words.
  double dist_ratio_test_factor_;  // The ratio of minimum distance and second
                                   // minimum distance below which the match is
                                   // rejected.
  int search_radius_;
  double search_view_dir_factor_low_;   // The searching factor when viewing
                                        // direction (aka. the angle) is low.
  double search_view_dir_factor_high_;  // The searching factor when viewing
                                        // direction (aka. the angle) is high.
  int max_n_feats_;  // Maximum number of features to be detected when a new
                     // frame is created.
  int init_min_n_feats_;    // Minimun number of features needed to do map
                            // initialization.
  int init_min_n_matches_;  // Minimum number of matches needed to do map
                            // initialization.
  int init_min_n_inlier_matches_;  // Minimum number of inlier matches survived
                                   // from global bundle adjustment during map
                                   // initialization.
  int min_n_feats_;
  int min_n_matches_;
  int min_n_inlier_matches_;
  double weight_factor_;  //
  int new_kf_interval_;
  int max_n_kfs_in_map_;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_CONFIG_H_
