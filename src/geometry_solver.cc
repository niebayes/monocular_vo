#include "mono_slam/geometry_solver.h"

#include "utils/math_utils.h"

namespace mono_slam {

void findFundamentalRansac(const Frame::Ptr& frame_1, const Frame::Ptr& frame_2,
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
      hypo_set.insert(math_utils::uniform_random_int(0, num_valid_matches - 1));
    // Collect matched feature correspondences.
    MatXX pts_1(2, 8), pts_2(2, 8);
    int c = 0;
    for (int i : hypo_set) {
      pts_1.col(c) = feats_1[valid_matches[i].first]->pt_;
      pts_2.col(c) = feats_2[valid_matches[i].second]->pt_;
      ++c;
    }

    // Compute fundamental matrix using normalized eight-point algorithm.
    geometry::normalizedFundamental8Point(pts_1.colwise().homogeneous(),
                                          pts_2.colwise().homogeneous(), F);
    vector<bool> inlier_mask;
    const int score = GeometrySolver::evaluateFundamentalScore(
        feats_1, feats_2, F, valid_matches, inlier_mask);
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

int evaluateFundamentalScore(const Frame::Features& feats_1,
                             const Frame::Features& feats_2, const Mat33& F,
                             const vector<pair<int, int>>& valid_matches,
                             vector<bool>& inlier_mask,
                             const double noise_sigma = 1.0) {
  const int num_valid_matches = valid_matches.size();
  inlier_mask.assign(num_valid_matches, false);

  for (int i = 0; i < num_valid_matches; ++i) {
    const Feature::Ptr& feat_1 = feats_1[valid_matches[i].first];
    const Feature::Ptr& feat_2 = feats_2[valid_matches[i].second];
    // FIXME Left multiply with F annd right multiply with F.
    const double dist_1 =
        geometry::pointToEpiLineDist(feat_1->pt_, feat_2->pt_, F, true);
    const double dist_2 =
        geometry::pointToEpiLineDist(feat_1->pt_, feat_2->pt_, F, false);
    const double chi2_one_degree = 3.841,
                 inv_sigma2 = 1. / (noise_sigma * noise_sigma);
    if (dist_1 * dist_1 * inv_sigma2 > chi2_one_degree ||
        dist_2 * dist_2 * inv_sigma2 > chi2_one_degree)
      continue;
    inlier_mask[i] = true;
  }
  return std::count(inlier_mask.cbegin(), inlier_mask.cend(), true);
}

bool findRelativePoseRansac(const Frame::Ptr& frame_1,
                            const Frame::Ptr& frame_2, const Mat33& F,
                            const vector<pair<int, int>>& inlier_matches,
                            SE3& relative_pose, vector<Vec3>& points,
                            vector<bool>& triangulate_mask,
                            const double noise_sigma = 1.0,
                            const int min_num_triangulated = 50,
                            const double min_parallax = 1.0) {
  // Obtain essential matrix.
  const Mat33& K = frame_1->cam_->K();
  const Mat33 E = K.transpose() * F * K;
  vector<Mat33> Rs;
  vector<Vec3> ts;
  geometry::decomposeEssential(E, Rs, ts);

  // Evaluate all combinations of R and t and select the best one.

  int best_score = 0;                  // Number of good triangulated points.
  Mat33 best_R;                        // R corresponding to highest score.
  Vec3 best_t;                         // t corresponding to highest score.
  vector<Vec3> best_points;            // Points corresponding to highest score.
  vector<bool> best_triangulate_mask;  // Mark which inlier match produces good
                                       // triangulated point.
  double best_median_parallax = 0.;
  for (Mat33& R : Rs) {
    for (Vec3& t : ts) {
      // Evaluate score of the current R and t.
      vector<Vec3> points_;
      vector<bool> triangulate_mask_;
      double median_parallax;
      const int score = GeometrySolver::evaluatePoseScore(
          R, t, frame_1->feats_, frame_2->feats_, inlier_matches, K, points_,
          triangulate_mask_, median_parallax, 2 * noise_sigma);
      if (score > best_score) {
        best_score = score;
        best_R = R;
        best_t = t;
        best_points = points_;
        best_triangulate_mask = triangulate_mask_;
        best_median_parallax = median_parallax;
      }
    }
  }

  // Obtain result.
  if (best_score < min_num_triangulated || best_median_parallax < min_parallax)
    return false;
  relative_pose = SE3(best_R, best_t);
  points = best_points;
  triangulate_mask = best_triangulate_mask;
  return true;
}

int evaluatePoseScore(const Mat33& R, const Vec3& t,
                      const Frame::Features& feats_1,
                      const Frame::Features& feats_2,
                      const vector<pair<int, int>>& inlier_matches,
                      const Mat33& K, vector<Vec3>& points,
                      vector<bool>& triangulate_mask, double& median_parallax,
                      const double repr_tolerance2,
                      const double min_parallax = 1.0) {
  // Initialize variables.
  const int num_inlier_matches = inlier_matches.size();
  points.assign(num_inlier_matches, Vec3{});  // Triangulated points.
  triangulate_mask.assign(num_inlier_matches, false);
  vector<double> cos_parallaxes;  // Cosine of parallaxes.
  cos_parallaxes.reserve(num_inlier_matches);

  // Obtain camera matices.
  // Left camera frame is fixed as world frame. Hence the world frame of the
  // triangulated points is the left camera frame.
  const Mat34 M_1 = K * Mat34::Identity();
  const Vec3 C_1 = Vec3::Zero();  // Left camera center.
  const Mat34 M_2 = math_utils::kRt2mat(K, R, t);
  const Vec3 C_2 = -R.transpose() * t;  // Right camera center.

  // Iterate all inlier matches and accumulate all good points.
  int num_good_points = 0;
  for (int i = 0; i < num_inlier_matches; ++i) {
    const Feature::Ptr& feat_1 = feats_1[inlier_matches[i].first];
    const Feature::Ptr& feat_2 = feats_2[inlier_matches[i].second];
    Vec3 point_1;
    geometry::triangulateLin(feat_1->pt_, feat_2->pt_, M_1, M_2, point_1);

    // Test 1: triangulated point is not infinitely far away as "infinite"
    // points can easily go to negative depth.
    if (point_1.array().isInf().any()) continue;
    // Test 2: triangulated point must have positive depth (in both cameras).
    const Vec3 point_2 = R * point_1 + t;
    if (point_1(2) <= 0. || point_2(2) <= 0.) continue;
    // Test 3: triangulated point must have sufficient parallax.
    const Vec3 bear_vec_1 = point_1 - C_1, bear_vec_2 = point_2 - C_2;
    const double cos_parallax =
        bear_vec_1.dot(bear_vec_2) / (bear_vec_1.norm() * bear_vec_2.norm());
    if (cos_parallax < std::cos(math_utils::degree2radian(min_parallax)))
      continue;
    // Test 4: the reprojection error must below the tolerance.
    const double repr_err_1 = geometry::computeReprErr(point_1, feat_1->pt_, K),
                 repr_err_2 = geometry::computeReprErr(point_2, feat_2->pt_, K);
    if (repr_err_1 >= repr_tolerance2 || repr_err_2 >= repr_tolerance2)
      continue;

    // Retain the point only if it passed all tests.
    points[i] = point_1;  // Retain points_1 since left camera is fixed as the
                          // world frame.
    triangulate_mask[i] = true;
    cos_parallaxes.push_back(cos_parallax);
    ++num_good_points;
  }

  // Obtain median parallax.
  if (!cos_parallaxes.empty()) {
    std::stable_sort(cos_parallaxes.begin(), cos_parallaxes.end());
    median_parallax = math_utils::radian2degree(
        std::acos(cos_parallaxes[num_good_points / 2]));
  } else
    median_parallax = 0.;

  return num_good_points;
}

bool P3PRansac(const Frame::Ptr& keyframe, const Frame::Ptr& frame,
               const vector<int>& matches, SE3& relative_pose,
               const double noise_sigma = 1.0) {
  // Retain only valid matches;
  const int num_matches = matches.size();
  vector<pair<int, int>> valid_matches;
  valid_matches.reserve(num_matches);
  for (int i = 0; i < num_matches; ++i)
    if (matches[i] != -1) valid_matches.push_back({i, matches[i]});
  const int num_valid_matches = valid_matches.size();

  // Obtain matched map points and features which form the 3D-2D correspondences
  // used in P3P.
  vector<MapPoint> points;  // Matched map points fetched from keyframe.
  points.reserve(num_valid_matches);
  vector<Frame::Ptr> feats_f;  // Matched features in frame.
  feats_f.reserve(num_valid_matches);
  for (int i = 0; i < num_valid_matches; ++i) {
    points.push_back(
        feat_utils::getPoint(keyframe->feats_[valid_matches[i].first]));
    feats_f.push_back(frame->feats_[valid_matches[i].second]);
  }

  // For the sake of efficiency, only few iterations of P3P are performed. The
  // qualify of the pose estimate will be further refined with pose graph
  // optimization.
  const int n_iters = Config::reloc_n_iters_p3p();
  bool has_found =
      false;  // At least one iteration of P3P solving is successful.
  SE3 best_T_c_w;
  int best_score = 0;
  for (int iter = 0; iter < n_iters; ++iter) {
    // Random sampling data for (Kneip) P3P.
    set<int> hypo_set;  // Hypothetical set to be populated.
    while (hypo_set.size() < 3)
      hypo.insert(math_utils::uniform_random_int(0, num_valid_matches - 1));
    Mat33 feature_vectors, world_points;
    int c = 0;
    for (int i : hypo_set) {
      feature_vectors.col(c) = frame->cam_->pixel2bear(feats_f[i]));
      world_points.col(c) = points[i].pos();
      ++c;
    }

    // Perform P3P solving.
    vector<SE3> T_c_w_vec;  // Four solutions.
    // Kneip P3P may fail in the case that all points are colinear.
    if (!geometry::P3PSolver::computePose(feature_vectors, world_points,
                                          T_c_w_vec)) {
      has_found = true;
      continue;
    }
    // Evaluate alternatively scores of four candidate poses and select the best
    // one with which the number of inliers passing the reprojection
    // thresholding test is maximized.
    SE3 best_T_c_w_i;
    const int best_score_i = GeometrySolver::evaluatePosesScore(
        T_c_w_vec, points, feats_f, keyframe->cam_->K(), best_T_c_w_i,
        2 * noise_sigma);
    if (best_score_i > best_score) {
      best_score = best_score_i;
      best_T_c_w = best_T_c_w_i;
    }
  }
  relative_pose = best_T_c_w;
  return has_found;
}

int evaluatePosesScore(const vector<SE3>& poses,
                       const vector<MapPoint::Ptr>& points,
                       const vector<Feature::Ptr>& feats, const Mat33& K,
                       SE3& best_pose, const double repr_tolerance2) {
  const int num_valid_matches = points.size();

  int max_num_inliers = 0;
  for (const SE3& pose : poses) {
    int num_inliers = 0;
    for (int i = 0; i < num_valid_matches; ++i) {
      vector<Vec3> points_c;  // Points in camera frame.
      points_c.reserve(num_valid_matches);
      std::transform(points.cbegin(), points.cend(),
                     std::back_inserter(points_c),
                     [&pose](const MapPoint::Ptr& point_w) {
                       return pose * point_w.pos();
                     });
      // FIXME Distortion is needed to be considered here?
      const double repr_err2 =
          geometry::computeReprErr(points_c[i], feats[i].pt_, K);
      if (repr_err2 < repr_tolerance2) ++num_inliers;
    }
    if (num_inliers > max_num_inliers) {
      max_num_inliers = num_inliers;
      best_pose = pose;
    }
  }
  return max_num_inliers;
}

namespace geometry {

void normalizedFundamental8Point(const MatXX& pts_1, const MatXX& pts_2,
                                 Mat33& F) {
  CHECK_EQ(pts_1.cols(), pts_2.cols());
  const int num_pts = pts_1.cols();
  CHECK_GE(num_pts, 8);

  // Normalize points.
  MatXX normalized_pts_1, normalized_pts_2;
  Mat33 T_1, T_2;
  normalizePoints(pts_1, normalized_pts_1, T_1);
  normalizePoints(pts_2, normalized_pts_2, T_2);

  // Find F' using normalized point correspondences.
  fundamental8Point(normalized_pts_1, normalized_pts_2, F);

  // Obtain unnormalized F from F'.
  F = (T_2.transpose() * F * T_1).eval();
}

void fundamental8Point(const MatXX& pts_1, const MatXX& pts_2, Mat33& F) {
  CHECK_EQ(pts_1.cols(), pts_2.cols());
  const int num_pts = pts_1.cols();
  CHECK_GE(num_pts, 8);

  MatXX A(num_pts, 9);
  // Vectorization trick: AXB = C -> (B' kron A) * vec(X) = vec(C);
  for (int i = 0; i < num_pts; ++i) {
    // FIXME Error in calling this function.
    // A.row(i) = Eigen::kroneckerProduct<Vec3, Vec3>(pts_1.col(i),
    // pts_2.col(i))
    //                .transpose()
    //                .eval();
  }
  auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Vec9& F_vec = svd.matrixV().rightCols(1);
  F.col(0) = F_vec.segment<3>(0);
  F.col(1) = F_vec.segment<3>(3);
  F.col(2) = F_vec.segment<3>(6);

  // Enforce the det(F) = 0 constraint.
  auto F_svd = F.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  Mat33 Sigma = F_svd.singularValues().asDiagonal();
  Sigma(2, 2) = 0.;
  F = F_svd.matrixU() * Sigma * F_svd.matrixV().transpose();
}

void decomposeEssential(const Mat33& E, vector<Mat33>& Rs, vector<Vec3>& ts) {
  // The four possible decompositions are encoded in the SVD of E.
  auto svd = E.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Mat33 U = svd.matrixU(), V = svd.matrixV();
  const Mat33 W = (Mat33() << 0., -1., 0., 1., 0., 0., 0., 0., 1.).finished();
  Rs.resize(2);
  ts.resize(2);
  Rs[0] = U * W * V.transpose();
  Rs[1] = U * W.transpose() * V.transpose();

  // Check if the decomposed Rs are valid rotation matrix (i.e. det(R) = +1).
  // If not, simply invert the sign.
  Rs[0] = (Rs[0].determinant() == 1) ? Rs[0].eval() : -Rs[0].eval();
  Rs[1] = (Rs[1].determinant() == 1) ? Rs[1].eval() : -Rs[1].eval();

  // Translations are encoded in the last column of U.
  // The two possible translations are +u3 and -u3.
  const Vec3& u3 = U.rightCols(1);
  ts[0] = u3;
  ts[1] = -u3;
}

void normalizePoints(const MatXX& pts, MatXX& normalized_pts, Mat33& T) {
  const int num_pts = pts.cols();
  CHECK_GE(num_pts, 0);

  // Mean.
  const Vec2 mean = pts.topRows(2).rowwise().sum();
  // Rescale factor.
  const double scale =
      std::sqrt(2. / (pts.topRows(2) - mean).squaredNorm() / num_pts);
  // Normalization matrix.
  T << scale, 0., -scale * mean.x(), 0., scale, -scale * mean.y(), 0., 0., 1.;
  // Perform normalization.
  normalized_pts = T * pts;
}

void triangulateLin(const Vec2& pt_1, const Vec2& pt_2, const Mat34& M_1,
                    const Mat34& M_2, Vec3& point) {
  //! A could be [6 x 4] or [4 x 4].
  MatXX A(6, 4);
  A.topRows(3) = geometry::to_skew(pt_1.homogeneous()) * M_1;
  A.bottomRows(3) = geometry::to_skew(pt_2.homogeneous()) * M_2;

  // Compute 3D points using SVD.
  auto svd = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  point = svd.matrixV().rightCols(1).colwise().hnormalized();
}

double computeReprErr(const Vec3& point, const Vec2& pt, const Mat33& K) {
  const Vec3& repr_point = K * point;
  // Perspective division.
  const double repr_x = repr_point(0) / repr_point(2);
  const double repr_y = repr_point(1) / repr_point(2);
  return ((pt.x() - repr_x) * (pt.x() - repr_x) +
          (pt.y() - repr_y) * (pt.y() - repr_y));
}

double pointToEpiLineDist(const Vec2& pt_1, const Vec2& pt_2,
                          const Mat33& F_2_1, const bool reverse = false) {
  if (!reverse) {
    // Epipolar line in image 2.
    const Vec3 epi_line = F_2_1 * pt_1.homogeneous();
    const double &a = epi_line(0), &b = epi_line(1), &c = epi_line(2);
    // Return distance between image point and epipolar line in image 2.
    return (std::abs((a * pt_2.x() + b * pt_2.y() + c)) /
            std::sqrt(a * a + b * b));
  } else {
    // Epipolar line in image 1.
    const Vec3 epi_line = pt_2.homogeneous().transpose() * F_2_1;
    const double &a = epi_line(0), &b = epi_line(1), &c = epi_line(2);
    // Return distance between image point and epipolar line in image 1.
    return (std::abs((a * pt_1.x() + b * pt_1.y() + c)) /
            std::sqrt(a * a + b * b));
  }
}

Mat33 getFundamentalByPose(const Frame::Ptr& frame_1,
                           const Frame::Ptr& frame_2) {
  const SE3 T_2_1 = frame_2->pose() * frame_1->pose().inverse();
  const Mat33 &K1 = frame_1->cam_->K(), &K2 = frame_2->cam_->K();
  return K1.transpose().inverse() * to_skew(T_2_1.translation()) *
         T_2_1.rotationMatrix() * K2.inverse();
}

Mat33 to_skew(const Vec3& vec) {
  Mat33 skew_mat;
  skew_mat << 0., -vec(2), vec(1), vec(2), 0., -vec(0), -vec(1), vec(0), 0.;
  return skew_mat;
}

}  // namespace geometry
}  // namespace mono_slam