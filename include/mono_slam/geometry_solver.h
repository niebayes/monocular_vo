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
                                    const int max_num_iterations = 200,
                                    const bool adaptive_iterations = true);

  // Evaluate the score of Fundamental matrix by computing reprojection error.
  static double EvaluateFundamentalScore(const Mat33& F,
                                         vector<bool>& inlier_mask,
                                         const double sigma = 1.0);

  static void ExtractRelativePoseRansac(const Mat33& F);

  static double EvaluatePoseScore();

  static void Triangulate();

  static void P3PRansac();

  static double PointsToEpipolarLineDistance();
};


namespace init_utils {

// Generate uniformly distributed random integer number in range [low, high].
int uniform_random_int(const int low, const int high);

}  // namespace init_utils
}  // namespace mono_slam

#endif  // MONO_SLAM_GEOMETRY_SOLVER_H_