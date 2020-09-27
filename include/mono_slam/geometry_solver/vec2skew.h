#ifndef MONO_SLAM_GEOMETRY_SOLVER_VEC2SKEW_H_
#define MONO_SLAM_GEOMETRY_SOLVER_VEC2SKEW_H_

#include "mono_slam/common_include.h"

namespace geometry {

inline Mat33 to_skew(const Vec3& vec) {
  Mat33 skew_mat;
  skew_mat << 0., -vec(2), vec(1), vec(2), 0., -vec(0), -vec(1), vec(0), 0.;
  return skew_mat;
}

}  // namespace geometry

#endif  // MONO_SLAM_GEOMETRY_SOLVER_VEC2SKEW_H_