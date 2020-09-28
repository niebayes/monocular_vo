#include "mono_slam/common_include.h"
#include "mono_slam/geometry_solver/vec2skew.h"

namespace geometry {

void LinearTriangulation(const Vec2& pt_1, const Vec2& pt_2, const Mat34& M_1,
                         const Mat34& M_2, Vec3& point) {
  //! A could be [6 x 4] or [4 x 4].
  MatXX A(6, 4);
  A.topRows(3) = geometry::to_skew(pt_1.homogeneous()) * M_1;
  A.bottomRows(3) = geometry::to_skew(pt_2.homogeneous()) * M_2;

  // Compute 3D points using SVD.
  auto svd = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  point = svd.matrixV().rightCols(1).colwise().hnormalized();
}

}  // namespace geometry
