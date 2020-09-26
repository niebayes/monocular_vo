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

  // Alternatively perform RANSAC to find the best F.
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

void FindRelativePose(const Frame::Ptr& frame_1, const Frame::Ptr& frame_2,
                      const Mat33& F,
                      const unordered_map<int, int>& inlier_matches,
                      SE3& relative_pose, vector<Vec3>& points,
                      vector<bool>& triangulate_mask,
                      const double noise_sigma = 1.0,
                      const int min_num_triangulated = 50,
                      const double min_parallax = 1.0, ) {
  // Obtain essential matrix.
  const Mat33& K = frame_1->cam_->K();
  const Mat33 E = K.transpose() * F * K;
  Mat36 Rs;
  Mat32 ts;
  geometry::DecomposeEssential(E, Rs, ts);

  // Evaluate all combination of R and t and select the best one.
  Mat33 best_R;  // best R.
  Vec3 best_t;   // best t.
  int best_score = 0;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      const Mat33& R = Rs.block<3, 3>(0, 3 * i);
      const Vec3& t = ts.col(j);

      // Evaluate score of the current R and t.
      vector<Vec3> points_;
      vector<bool> triangulate_mask_;
      double median_parallax;
      const int score = EvaluatePoseScore(
          R, t, frame_1->feats_, frame_2->feats_, inlier_matches, K, points_,
          triangulate_mask_, median_parallax, 2 * noise_sigma);
      if (score > best_score) {
        best_score = score;
        best_R = R;
        best_t = t;
      }
    }
  }
}

static int EvaluatePoseScore(const Mat33& R, const Vec3& t,
                             const Frame::Features& feats_1,
                             const Frame::Features& feats_2,
                             const unordered_map<int, int>& inlier_matches,
                             const Mat33& K, vector<Vec3>& points,
                             vector<bool>& triangulate_mask,
                             double& median_parallax,
                             const double reproj_tolerance,
                             const double min_parallax = 1.0) {
  // Initialize variables.
  const num_inlier_matches = inlier_matches.size();
  points.resize(num_inlier_matches);
  triangulate_mask.assign(num_inlier_matches, false);
  vector<double> cos_parallaxes; 
  cos_parallaxes.reserve(num_inlier_matches);

  // Obtain camera matices.
  // Left camera frame is fixed as world frame. Hence the world frame of the
  // triangulated points is the left camera frame.
  // TODO(bayes) Wrap the logic into a function and store it to Camera class.
  const Mat34 M_1 = K * Mat34::Identity();
  const Vec3 C_1 = Vec3::Zeros();  // Left camera center.
  Mat34 M_2;
  M_2.leftCols(3) = K * R;
  M_2.rightCols(1) = K * t;
  const Vec3 C_2 = -R.transpose() * t;

  // Iterate all inlier matches and accumulate all good points.
  for (int i = 0; i < num_inlier_matches; ++i) {
    const Feature::Ptr& feat_1 = feats_1[i];
    const Feature::Ptr& feat_2 = feats_2[i];
    Vec3 point_1;
    geometry::Triangulate(feat_1->pt_, feat_2->pt_, M_1, M_2, point_1);

    // Test 1: triangulated point is not infinitely far away as "infinite"
    // points can easily go to negative depth.
    if (point.isInf().any()) continue;
    // Test 2: triangulated point must have positive depth (in both cameras).
    const Vec3 point_2 = R * point_1 + t;
    if (point_1(2) <= 0 || point_2(2) <= 0) continue;
    // Test 3: triangulated point must have sufficient parallax.
    const Vec3 bear_vec_1 = point_1 - C_1, bear_vec_2 = point_2 - C_2;
    const double cos_parallax =
        bear_vec_1.dot(bear_vec_2) / bear_vec_1.norm() * bear_vec_2.norm();
    if (cos_parallax < std::cos(init_utils::degree2radian(min_parallax)))
      continue;
    // Test 4: the reprojection error must below the tolerance.
    const double reproj_error_1 = geometry::ComputeReprojectionError(
                     point_1, feat_1->pt_, K),
                 reproj_error_2 = geometry::ComputeReprojectionError(
                     point_2, feat_2->pt_, K);
    if (reproj_error_1 > reproj_tolerance || reproj_error_2 > reproj_tolerance)
      continue;
    
    // Retain the point only if it passed all tests.
    const int& idx = inlier_matches[i].first;
    points[idx] = point;
    triangulate_mask[idx] = true;
    cos_parallaxes.push_back(cos_parallax);
  }

  return num_good_points;
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

double degree2radian(const double degree) { return degree * 180.0 / EIGEN_PI; }

}  // namespace init_utils
}  // namespace geometry