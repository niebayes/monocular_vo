#include "mono_slam/common_include.h"
#include "mono_slam/system.h"

using namespace mono_slam;

int main() { return EXIT_SUCCESS; }

void LinearTriangulation(const MatXX& pts_1, const MatXX& pts_2,
                         const Mat34& M_1, const Mat34& M_2, MatXX& points) {
  CHECK_EQ(pts_1.cols(), pts_2.cols());
  const int num_pts = pts_1.cols();
  points.resize(3, num_pts);

  for (int i = 0; i < num_pts; ++i) {
    MatXX A(6, 4);
  }
}