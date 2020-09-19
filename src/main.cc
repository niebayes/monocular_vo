#include <iostream>
#include <vector>

#include "DBoW3/DBoW3.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/types/slam3d/g2o_types_slam3d_api.h"

int main(int argc, char** argv) {
  // CLI arguments:
  // image_file, timestamp_file, config_file, vocabulary_file

  // Create SLAM system
  // Pass in config_file, vocabulary_file
  // Ready all threads

  // Main loop
  // Pass images frame by frame along with the timestamp to Tracking thread.
  // Be careful the delay of timestamp.

  // Shutdown system.

  // Save trajectory and all map points.

  return EXIT_SUCCESS;
}