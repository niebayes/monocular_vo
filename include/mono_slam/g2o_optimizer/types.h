#ifndef MONO_SLAM_G2O_OPTIMIZER_TYPES_H_
#define MONO_SLAM_G2O_OPTIMIZER_TYPES_H_

// FIXME which definition is in which file?
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/types/slam3d/g2o_types_slam3d_api.h"  // g2o::VertexSBAPointXYZ
#include "mono_slam/common_include.h"
#include "mono_slam/feature.h"
#include "mono_slam/frame.h"

#define CAMERA_PARAMETER_ID 0

namespace g2o_types {

class Frame;
class Feature;

// Solver type typedefs.
using BlockSolver = g2o::BlockSolver_6_3;
using LinearSolver = g2o::LinearSolverCSparse<BlockSolver::PoseMatrixType>;

// Edge and vertex typedefs.
using EdgeObs = g2o::EdgeProjectXYZ2UV;
using VertexFrame = g2o::VertexSE3Expmap;
using VertexPoint = g2o::VertexSBAPointXYZ;

struct EdgeContainer {
  sptr<EdgeObs> e_obs_ = nullptr;
  Frame::Ptr keyframe_ = nullptr;
  Feature::Ptr feat_ = nullptr;
  g2oEdgeContainer(sptr<EdgeObs> e_obs, Frame::Ptr keyframe, Feature::Ptr feat)
      : e_obs_(e_obs), keyframe_(keyframe), feat_(feat) {}
};

}  // namespace g2o_types

#endif  // MONO_SLAM_G2O_OPTIMIZER_TYPES_H_