#include "Eigen/Core"
#include "glog/logging.h"

using namespace Eigen;

namespace geometry {

void NormalizePoints(const MatXX& pts, MatXX& normalized_pts, Mat33& T) {
  const int num_pts = pts.cols();
  CHECK_GE(num_pts, 0);

  // Mean.
  const Vec2 mean = pts.topRows(2).rowwise().sum();
  // Rescale factor.
  const double scale =
      std::sqrt(2.0 / (pts.topRows(2) - mean).squaredNorm() / num_pts);
  // Normalization matrix.
  T << scale, 0., -scale * mean.x(), 
       0., scale, -scale * mean.y(),
       0., 0., 1.;
  // Perform normalization.
  normalized_pts = T * pts;
}

}  // namespace geometry
