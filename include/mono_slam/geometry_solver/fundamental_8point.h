#include "eigen3/unsupported/Eigen/KroneckerProduct"
#include "mono_slam/common_include.h"

namespace geometry {

void Fundamental8Point(const MatXX& pts_1, const MatXX& pts_2, Mat33& F) {
  CHECK_EQ(pts_1.cols(), pts_2.cols());
  const int num_pts = pts_1.cols();
  CHECK_GE(num_pts, 8);

  MatXX A(num_pts, 9);
  // Vectorization trick: AXB = C -> (B' kron A) * vec(X) = vec(C);
  for (int i = 0; i < num_pts; ++i) {
    A.row(i) = KroneckerProduct<Vec3, Vec3>(pts_1.col(i), pts_2.col(i))
                   .transpose()
                   .eval();
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

}  // namespace geometry