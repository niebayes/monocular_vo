#include "mono_slam/g2o_optimizer.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/types/slam3d/g2o_types_slam3d_api.h"

namespace mono_slam {

void GlobalBundleAdjustment(const Map::Keyframes& keyframes,
                            const Map::MapPoints& points, Map::Ptr& map,
                            const int num_iterations) {
  //
}

}  // namespace mono_slam