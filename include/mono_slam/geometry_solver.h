#ifndef MONO_SLAM_GEOMETRY_SOLVER_H_
#define MONO_SLAM_GEOMETRY_SOLVER_H_

#include "mono_slam/common_include.h"
#include "mono_slam/frame.h"

namespace mono_slam {

class GeometrySolver {
 public:
  // Find fundamental matrix using eight-point algorithm in a RANSAC scheme.
  static void findFundamentalRansac(const Frame::Ptr& frame_1,
                                    const Frame::Ptr& frame_2,
                                    const vector<int>& matches, Mat33& F,
                                    vector<pair<int, int>>& inlier_matches,
                                    const double noise_sigma = 1.0,
                                    const int max_num_iterations = 200,
                                    const bool adaptive_iterations = true);

  // Evaluate the score of Fundamental matrix by computing reprojection error.
  static int evaluateFundamentalScore(const Frame::Features& feats_1,
                                      const Frame::Features& feats_2,
                                      const Mat33& F,
                                      const vector<pair<int, int>>& matches,
                                      vector<bool>& inlier_mask,
                                      const double noise_sigma = 1.0);

  // Find the best relative pose by decomposing essential matrix in a RANSAC
  // scheme.
  static bool findRelativePoseRansac(
      const Frame::Ptr& frame_1, const Frame::Ptr& frame_2, const Mat33& F,
      const vector<pair<int, int>>& inlier_matches, SE3& relative_pose,
      vector<Vec3>& points, vector<bool>& triangulate_mask,
      const double noise_sigma = 1.0, const int min_num_triangulated = 50,
      const double min_parallax = 1.0);

  // Evaluate the score of pose by counting number of good triangulated points.
  static int evaluatePoseScore(const Mat33& R, const Vec3& t,
                               const Frame::Features& feats_1,
                               const Frame::Features& feats_2,
                               const vector<pair<int, int>>& inlier_matches,
                               const Mat33& K, vector<Vec3>& points,
                               vector<bool>& triangulate_mask,
                               double& median_parallax,
                               const double repr_tolerance2,
                               const double min_parallax = 1.0);

  // Find the best relative pose frame relocalization candidate keyframe to
  // quering frame.
  static bool P3PRansac(const Frame::Ptr& keyframe, const Frame::Ptr& frame,
                        const vector<int>& matches, SE3& relative_pose,
                        const double noise_sigma = 1.0);

  // Evaluate the scores of the four solutions obtained from Kneip P3P. The best
  // score among them is returned.
  static int evaluatePosesScore(const vector<SE3>& poses,
                                const vector<MapPoint::Ptr>& points,
                                const vector<Feature::Ptr>& feats,
                                const Mat33& K, SE3& best_pose,
                                const double repr_tolerance2);
};

namespace geometry {

void normalizedFundamental8Point(const MatXX& pts_1, const MatXX& pts_2,
                                 Mat33& F);

void fundamental8Point(const MatXX& pts_1, const MatXX& pts_2, Mat33& F);

void decomposeEssential(const Mat33& E, vector<Mat33>& Rs, vector<Vec3>& ts);

// FIXME Inline at here and define at another place, inline still works?
inline void normalizePoints(const MatXX& pts, MatXX& normalized_pts, Mat33& T);

inline void triangulateLin(const Vec2& pt_1, const Vec2& pt_2, const Mat34& M_1,
                           const Mat34& M_2, Vec3& point);

inline double computeReprErr(const Vec3& point, const Vec2& pt, const Mat33& K);

inline double pointToEpiLineDist(const Vec2& pt_1, const Vec2& pt_2, const Mat33& F_2_1, const bool reverse =false);

inline Mat33 getFundamentalByPose(const Frame::Ptr& frame_1,
                                  const Frame::Ptr& frame_2);

inline Mat33 to_skew(const Vec3& vec);

}  // namespace geometry
}  // namespace mono_slam

#include "mono_slam/geometry_solver/kneip_p3p.h"

#endif  // MONO_SLAM_GEOMETRY_SOLVER_H_