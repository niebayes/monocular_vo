#include "mono_slam/common_include.h"

namespace geometry {

inline Mat33 to_skew(const Vec3& vec) {
  Mat33 skew_mat;
  skew_mat << 0., -vec(2), vec(1), vec(2), 0., -vec(0), -vec(1), vec(0), 0.;
  return skew_mat;
}

}  // namespace geometry
