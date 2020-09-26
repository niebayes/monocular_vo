#ifndef MONO_SLAM_GEOMETRY_SOLVER_DECOMPOSE_ESSENTIAL_H_
#define MONO_SLAM_GEOMETRY_SOLVER_DECOMPOSE_ESSENTIAL_H_

#include "mono_slam/common_include.h"

namespace geometry {

void DecomposeEssential(const Mat33& E, Mat36& Rs, Mat32& ts) {
  // The four possible decompositions are encoded in the SVD of E.
  auto svd = E.jacobiSvd(Eigen::ComputeThinU | Eigene::ComputeThinV);
  const Mat33 U = svd.matrixU(), V = svd.matrixV();
  const Mat33 W = (Mat33() << 0., -1., 0., 1., 0., 0., 0., 0., 1.).finished();
  Mat33 &R1 = Rs.leftCols(3), &R2 = Rs.rightCols(3);
  R1 = U * W * V.transpose();
  R2 = U * W.transpose() * V.transpose();

  // Check if the decomposed Rs are valid rotation matrix (i.e. det(R) = +1).
  // If not, simply invert the sign.
  R1 = (R1.determinant() == 1) ? R1.eval() : -R1.eval();
  R2 = (R2.determinant() == 1) ? R2.eval() : -R2.eval();

  // Translations are encoded in the last column of U.
  // The two possible translations are +u3 and -u3.
  const Vec3& u3 = U.rightCols(1);
  ts.col(0) = u3;
  ts.col(1) = -u3;
}

}  // namespace geometry

#endif  // MONO_SLAM_GEOMETRY_SOLVER_DECOMPOSE_ESSENTIAL_H_
