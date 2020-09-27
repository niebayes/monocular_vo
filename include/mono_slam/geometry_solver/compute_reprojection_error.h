#ifndef MONO_SLAM_GEOMETRY_SOLVER_COMPUTE_REPROJECTION_ERROR_
#define MONO_SLAM_GEOMETRY_SOLVER_COMPUTE_REPROJECTION_ERROR_

#include <cmath>

#include "mono_slam/common_include.h"

namespace geometry {

double ComputeReprojectionError(const Vec3& point, const Vec2& pt,
                                const Mat33& K) {
  const Vec3& reproj_point = K * point;
  // Perspective division.
  const double reproj_x = reproj_point(0) / reproj_point(2);
  const double reproj_y = reproj_point(1) / reproj_point(2);
  return ((pt.x() - reproj_x) * (pt.x() - reproj_x) +
          (pt.y() - reproj_y) * (pt.y() - reproj_y));
}

}  // namespace geometry

#endif  // MONO_SLAM_GEOMETRY_SOLVER_COMPUTE_REPROJECTION_ERROR_