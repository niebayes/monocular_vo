#ifndef MONO_SLAM_GEOMETRY_SOLVER_H_
#define MONO_SLAM_GEOMETRY_SOLVER_H_

#include "mono_slam/common_include.h"

namespace mono_slam {

class GeometrySolver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = sptr<GeometrySolver>;
};

#include "mono_slam/geometry_solver/normalized_fundamental_8point.h"
#include "mono_slam/geometry_solver/kneip_p3p.h"
#include "mono_slam/geometry_solver/linear_triangulation.h"
#include "mono_slam/geometry_solver/points_to_epipolar_line_distance.h"

}  // namespace mono_slam

#endif  // MONO_SLAM_GEOMETRY_SOLVER_H_