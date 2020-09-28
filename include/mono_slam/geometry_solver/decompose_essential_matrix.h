#ifndef MONO_SLAM_GEOMETRY_SOLVER_DECOMPOSE_ESSENTIAL_H_
#define MONO_SLAM_GEOMETRY_SOLVER_DECOMPOSE_ESSENTIAL_H_

#include "mono_slam/common_include.h"

namespace geometry {

void DecomposeEssential(const Mat33& E, vector<Mat33>& Rs, vector<Vec3>& ts) {
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

}  // namespace geometry

#endif  // MONO_SLAM_GEOMETRY_SOLVER_DECOMPOSE_ESSENTIAL_H_
