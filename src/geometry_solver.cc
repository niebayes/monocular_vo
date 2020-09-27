#include "mono_slam/geometry_solver.h"

#include "mono_slam/camera.h"
#include "mono_slam/geometry_solver/decompose_essential_matrix.h"
#include "mono_slam/geometry_solver/disambiguate_poses.h"
#include "mono_slam/geometry_solver/kneip_p3p.h"
#include "mono_slam/geometry_solver/linear_triangulation.h"
#include "mono_slam/geometry_solver/normalized_fundamental_8point.h"
#include "mono_slam/geometry_solver/points_to_epipolar_line_distance.h"

namespace mono_slam {

void FindFundamentalRansac(const Frame::Ptr& frame_1, const Frame::Ptr& frame_2,
                           const vector<int>& matches, Mat33& F,
                           vector<pair<int, int>>& inlier_matches,
                           const int max_num_iterations = 200,
                           const bool adaptive_iterations = true) {
  // Retain only valid matches.
  const int num_matches = matches.size();
  vector<pair<int, int>> valid_matches;
  valid_matches.reserve(num_matches);
  for (int i = 0; i < num_matches; ++i)
    if (matches[i] != -1) valid_matches.push_back(make_pair(i, matches[i]));
  const int num_valid_matches = valid_matches.size();
  CHECK_GE(num_valid_matches, 8);

  const Frame::Features& feats_1 = frame_1->feats_;
  const Frame::Features& feats_2 = frame_2->feats_;

  // Alternatively perform RANSAC to find the best F.
  int best_score = 0;
  Mat33 best_F;
  vector<bool> best_inlier_mask(num_valid_matches, false);
  int num_iterations = adaptive_iterations ? std::numeric_limits<int>::max()
                                           : max_num_iterations;
  for (int iter = 0; iter < num_iterations; ++iter) {
    // Generate a hypothetical set used in RANSAC.
    std::set<int> hypo_set;
    while (hypo_set.size() < 8)
      hypo_set.insert(init_utils::uniform_random_int(0, num_valid_matches - 1));
    // Collect matched feature correspondences.
    MatXX pts_1(2, 8), pts_2(2, 8);
    int c = 0;
    for (int i : hypo_set) {
      pts_1.col(c) = feats_1[valid_matches[i].first]->pt_;
      pts_2.col(c) = feats_2[valid_matches[i].second]->pt_;
      ++c;
    }

    // Compute fundamental matrix using normalized eight-point algorithm.
    geometry::NormalizedFundamental8Point(pts_1.homogeneous(),
                                          pts_2.homogeneous(), F);
    vector<bool> inlier_mask;
    const int score = EvaluateFundamentalScore(feats_1, feats_2, F,
                                               valid_matches, inlier_mask);
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
          std::min(1.0 - max_num_inliers / num_valid_matches, 0.90);
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
  inlier_matches.reserve(num_valid_matches);
  for (int i = 0; i < num_valid_matches; ++i)
    if (best_inlier_mask[i]) inlier_matches.push_back(valid_matches[i]);
}

int EvaluateFundamentalScore(const Frame::Features& feats_1,
                             const Frame::Features& feats_2, const Mat33& F,
                             const vector<pair<int, int>>& valid_matches,
                             vector<bool>& inlier_mask,
                             const double noise_sigma = 1.0) {
  const int num_valid_matches = valid_matches.size();
  inlier_mask.assign(num_valid_matches, false);

  for (int i = 0; i < num_valid_matches; ++i) {
    const Feature::Ptr& feat_1 = feats_1[valid_matches[i].first];
    const Feature::Ptr& feat_2 = feats_2[valid_matches[i].second];
    const double dist_1 = geometry::PointToEpipolarLineDistance(feat_1->pt_, F);
    const double dist_2 = geometry::PointToEpipolarLineDistance(feat_2->pt_, F);
    const double chi2_one_degree = 3.841,
                 inv_sigma2 = 1.0 / (noise_sigma * noise_sigma);
    if (dist_1 * dist_1 * inv_sigma2 > chi2_one_degree ||
        dist_2 * dist_2 * inv_sigma2 > chi2_one_degree)
      continue;
    inlier_mask[i] = true;
  }
  return std::count(inlier_mask.cbegin(), inlier_mask.cend(), true);
}

bool FindRelativePoseRansac(const Frame::Ptr& frame_1,
                            const Frame::Ptr& frame_2, const Mat33& F,
                            const vector<pair<int, int>>& inlier_matches,
                            SE3& relative_pose, vector<Vec3>& points,
                            const double noise_sigma = 1.0,
                            const int min_num_triangulated = 50,
                            const double min_parallax = 1.0) {
  // Obtain essential matrix.
  const Mat33& K = frame_1->cam_->K();
  const Mat33 E = K.transpose() * F * K;
  Mat36 Rs;
  Mat32 ts;
  geometry::DecomposeEssential(E, Rs, ts);

  // Evaluate all combinations of R and t and select the best one.

  int best_score = 0;        // Number of good triangulated points.
  Mat33 best_R;              // R corresponding to highest score.
  Vec3 best_t;               // t corresponding to highest score.
  vector<Vec3> best_points;  // Points corresponding to highest score.
  double best_median_parallax = 0;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      const Mat33& R = Rs.block<3, 3>(0, 3 * i);
      const Vec3& t = ts.col(j);

      // Evaluate score of the current R and t.
      vector<Vec3> points_;
      double median_parallax;
      const int score = EvaluatePoseScore(
          R, t, frame_1->feats_, frame_2->feats_, inlier_matches, K, points_,
          median_parallax, 2 * noise_sigma);
      if (score > best_score) {
        best_score = score;
        best_R = R;
        best_t = t;
        best_points = points_;
        best_median_parallax = median_parallax;
      }
    }
  }

  // Obtain result.
  if (best_score < min_num_triangulated || best_median_parallax < min_parallax)
    return false;
  relative_pose = SE3(best_R, best_t);
  points = best_points;
}

int EvaluatePoseScore(const Mat33& R, const Vec3& t,
                      const Frame::Features& feats_1,
                      const Frame::Features& feats_2,
                      const vector<pair<int, int>>& inlier_matches,
                      const Mat33& K, vector<Vec3>& points,
                      double& median_parallax, const double reproj_tolerance2,
                      const double min_parallax = 1.0) {
  // Initialize variables.
  const num_inlier_matches = inlier_matches.size();
  points.assign(num_inlier_matches, Vec3{});  // Triangulated points.
  vector<double> cos_parallaxes;              // Cosine of parallaxes.
  cos_parallaxes.reserve(num_inlier_matches);

  // Obtain camera matices.
  // Left camera frame is fixed as world frame. Hence the world frame of the
  // triangulated points is the left camera frame.
  const Mat34 M_1 = K * Mat34::Identity();
  const Vec3 C_1 = Vec3::Zeros();  // Left camera center.
  const Mat34 M_2 = Camera::to_cam_mat(K, R, t);
  const Vec3 C_2 = -R.transpose() * t;  // Right camera center.

  // Iterate all inlier matches and accumulate all good points.
  for (int i = 0; i < num_inlier_matches; ++i) {
    const Feature::Ptr& feat_1 = feats_1[inlier_matches[i].first];
    const Feature::Ptr& feat_2 = feats_2[inlier_matches[i].second];
    Vec3 point_1;
    geometry::LinearTriangulation(feat_1->pt_, feat_2->pt_, M_1, M_2, point_1);

    // Test 1: triangulated point is not infinitely far away as "infinite"
    // points can easily go to negative depth.
    if (point_1.isInf().any()) continue;
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
    if (reproj_error_1 > reproj_tolerance2 ||
        reproj_error_2 > reproj_tolerance2)
      continue;

    // Retain the point only if it passed all tests.
    points[i] = points_1;  // Retain points_1 since left camera is fixed as the
                           // world frame.
    cos_parallaxes.push_back(cos_parallax);
  }

  // Count non-empty points.
  const int num_good_points =
      std::count_if(points.cbegin(), points.cend(),
                    [](Vec3 point) { return !point.empty(); });
  // Obtain the median parallax.
  if (!cos_parallaxes.empty()) {
    std::stable_sort(cos_parallaxes.begin(), cos_parallaxes.end());
    median_parallax = init_utils::radian2degree(
        std::acos(cos_parallaxes[num_good_points / 2]));
  } else
    median_parallax = 0.;

  return num_good_points;
}  // namespace mono_slam

namespace init_utils {

int uniform_random_int(const int low, const int high) {
  const unsigned seed =
      std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_int_distribution<int> distribution(low, high);
  auto dice_once = std::bind(distribution, generator);
  return dice_once();
}

double degree2radian(const double degree) { return degree * EIGEN_PI / 180.0; }

double radian2degree(const double radian) { return radian * 180.0 / EIGEN_PI; }

}  // namespace init_utils
}  // namespace mono_slam