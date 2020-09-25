#include "mono_slam/common_include.h"
#include "mono_slam/geometry_solver/vec2skew.h"

namespace geometry {

void LinearTriangulation(const MatXX& pts_1, const MatXX& pts_2,
                         const Mat34& M_1, const Mat34& M_2, MatXX& points) {
  CHECK_EQ(pts_1.cols(), pts_2.cols());
  const int num_pts = pts_1.cols();
  points.resize(4, num_pts);

  for (int i = 0; i < num_pts; ++i) {
    // A could be [6 x 4] or [4 x 4].
    MatXX A(6, 4);
    A.topRows(3) = to_skew(pts_1.col(i)) * M_1;
    A.bottomRows(3) = to_skew(pts_2.col(i)) * M_2;

    // Compute 3D points using SVD.
    auto svd = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    points.col(i) = svd.matrixV().rightCols(1);
  }
  // Enforce homogeneous coordinates (i.e. last element is 0 for each column.)
  points.topRows(3) /= points.bottomRows(1);
}

}  // namespace geometry