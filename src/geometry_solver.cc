#include "mono_slam/geometry_solver.h"
#include "mono_slam/geometry_solver/decompose_essential_matrix.h"
#include "mono_slam/geometry_solver/disambiguate_poses.h"
#include "mono_slam/geometry_solver/kneip_p3p.h"
#include "mono_slam/geometry_solver/linear_triangulation.h"
#include "mono_slam/geometry_solver/normalized_fundamental_8point.h"
#include "mono_slam/geometry_solver/points_to_epipolar_line_distance.h"

namespace geometry {

void FindFundamentalRansac(const Frame::Ptr& frame_1, const Frame::Ptr& frame_2,
                           const vector<int>& matches, Mat33& F,
                           unordered_map<int, int>& inlier_matches,
                           const int max_num_iterations = 200,
                           const bool adaptive_iterations = true) {
  const int num_matches = matches.size();
  CHECK_GE(num_matches, 8);
  const Frame::Features& feats_1 = frame_1->feats_;
  const Frame::Features& feats_2 = frame_2->feats_;

  double best_score = 0;
  Mat33 best_F;
  vector<bool> best_inlier_mask(num_matches, false);
  int num_iterations = adaptive_iterations ? std::numeric_limits<int>::max()
                                           : max_num_iterations;
  for (int iter = 0; iter < num_iterations; ++iter) {
    // Generate a hypothetical set used in RANSAC.
    std::set<int> hypo_set;
    while (hypo_set.size() < 8)
      hypo_set.insert(init_utils::uniform_random_int(0, num_matches - 1));
    // Collect matched feature correspondences.
    MatXX pts_1(3, 8), pts_2(3, 8);
    int c = 0;
    for (int i : hypo_set) {
      pts_1.col(c) = Vec3{feats_1[i]->pt_.x(), feats_1[i]->pt_.y(), 1.0};
      pts_2.col(c) =
          Vec3{feats_2[matches[i]]->pt_.x(), feats_2[matches[i]]->pt_.y(), 1.0};
      ++c;
    }

    // Compute fundamental matrix using normalized eight-point algorithm.
    geometry::NormalizedFundamental8Point(pts_1, pts_2, F);
    vector<bool> inlier_mask;
    const double score = EvaluateFundamentalScore(F, inlier_mask);
    if (score > best_score) {
      best_score = score;
      best_F = F;
      best_inlier_mask = inlier_mask;
    }

    // Adaptively change number of iterations.
    if (adaptive_iterations) {
      const int max_num_inliers =
          std::count(best_inlier_mask.cbegin(), best_inlier_mask.cend(), true);
      // Upper bound of outlier ratio is set to 0.90
      const double outlier_ratio =
          std::min(1.0 - max_num_inliers / num_matches, 0.90);
      // Confidence about how much matches are inliers.
      const double confidence = 0.95;
      num_iterations = std::log(1.0 - confidence) /
                       std::log(1.0 - std::pow(1.0 - outlier_ratio, 8));
      // Set the upper bound of number of iterations.
      num_iterations = std::min(num_iterations, max_num_iterations);
    }
  }

  // Obtain result.
  F = best_F;
  inlier_matches.reserve(num_matches);
  for (int i = 0; i < num_matches; ++i)
    if (best_inlier_mask[i]) inlier_matches.push_back(make_pair(i, matches[i]));
}

namespace init_utils {

int uniform_random_int(const int low, const int high) {
  const unsigned seed =
      std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_int_distribution<int> distribution(low, high);
  auto dice_once = std::bind(distribution, generator);
  return dice_once();
}

}  // namespace init_utils
}  // namespace geometry