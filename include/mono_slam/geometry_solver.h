#ifndef MONO_SLAM_GEOMETRY_SOLVER_H_
#define MONO_SLAM_GEOMETRY_SOLVER_H_

#include "mono_slam/common_include.h"

namespace mono_slam {

class GeometrySolver {
 public:
  // Find fundamental matrix using eight-point algorithm in a RANSAC scheme.
  static void FindFundamentalRansac(const Frame::Ptr& frame_1,
                                    const Frame::Ptr& frame_2,
                                    const vector<int>& matches, Mat33& F,
                                    unordered_map<int, int>& inlier_matches,
                                    const double noise_sigma = 1.0,
                                    const int max_num_iterations = 200,
                                    const bool adaptive_iterations = true);

  // Evaluate the score of Fundamental matrix by computing reprojection error.
  static double EvaluateFundamentalScore(const Mat33& F,
                                         vector<bool>& inlier_mask,
                                         const double noise_sigma = 1.0);

  static void FindRelativePose(const Frame::Ptr& frame_1,
                               const Frame::Ptr& frame_2, const Mat33& F,
                               const unordered_map<int, int>& inlier_matches,
                               SE3& relative_pose, vector<Vec3>& points,
                               vector<bool>& triangulate_mask,
                               const double noise_sigma = 1.0,
                               const int min_num_triangulated = 50,
                               const double min_parallax = 1.0);

  static int EvaluatePoseScore(const Mat33& R, const Vec3& t,
                               const Frame::Features& feats_1,
                               const Frame::Features& feats_2,
                               const unordered_map<int, int>& inlier_matches,
                               const Mat33& K, vector<Vec3>& points,
                               vector<bool>& triangulate_mask,
                               double& median_parallax,
                               const double reproj_tolerance);

  static void Triangulate();

  static void P3PRansac();

  static double PointsToEpipolarLineDistance();
};

namespace init_utils {

// Generate uniformly distributed random integer number in range [low, high].
int uniform_random_int(const int low, const int high);

// Transform degrees to radians.
double degree2radian(const double degree) {

}  // namespace init_utils
}  // namespace mono_slam

#endif  // MONO_SLAM_GEOMETRY_SOLVER_H_