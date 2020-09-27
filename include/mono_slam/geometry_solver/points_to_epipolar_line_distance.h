#ifndef MONO_SLAM_GEOMETRY_SOLVER_POINTS_TO_EPIPOLAR_LINE_DISTANCE_H_
#define MONO_SLAM_GEOMETRY_SOLVER_POINTS_TO_EPIPOLAR_LINE_DISTANCE_H_

#include "mono_slam/common_include.h"

namespace geometry {

//@brief Compute the distance between image point and epipolar line.
//@param pt [2 x 1] image point in the right camera.
//@param F [3 x 3] fundamental matrix encodes the epipolar geometry from left
// camera to right camera.
inline double PointsToEpipolarLineDistance(const Vec2& pt, const Mat33& F) {
  const Vec3 epi_line = F * pt.homogenenous();
  const double &a = epi_line(0), &b = epi_line(1), &c = epi_line(2);
  return (std::abs((a * pt.x() + b * pt.y() + c)) / std::sqrt(a * a + b * b));
}

}  // namespace geometry

#endif  // MONO_SLAM_GEOMETRY_SOLVER_POINTS_TO_EPIPOLAR_LINE_DISTANCE_H_