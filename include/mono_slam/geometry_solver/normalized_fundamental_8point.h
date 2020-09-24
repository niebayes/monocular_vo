#include "mono_slam/common_include.h"
#include "mono_slam/geometry_solver/fundamental_8point.h"
#include "mono_slam/geometry_solver/normalize_points.h"

namespace geometry {

void NormalizedFundamental8Point(const MatXX& pts_1, const MatXX& pts_2,
                                 Mat33& F) {
  CHECK_EQ(pts_1.cols(), pts_2.cols());
  const int num_pts = pts_1.cols();
  CHECK_GE(num_pts, 8);

  // Normalize points.
  MatXX normalized_pts_1, normalized_pts_2;
  Mat33 T_1, T_2;
  NormalizePoints(pts_1, normalized_pts_1, T_1);
  NormalizePoints(pts_2, normalized_pts_2, T_2);

  // Find F' using normalized point correspondences.
  Fundamental8Point(normalized_pts_1, normalized_pts_2, F);

  // Obtain unnormalized F from F'.
  F = (T_2.transpose() * F * T_1).eval();
}

}  // namespace geometry