#ifndef MONO_SLAM_UTILS_MATH_UTILS_H_
#define MONO_SLAM_UTILS_MATH_UTILS_H_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <vector>

#include "Eigen/Core"
#include "armadillo"
#include "glog/logging.h"
#include "mono_slam/config.h"
#include "sophus/se3.hpp"

using Vec3 = Eigen::Vector3d;
using Mat33 = Eigen::Matrix3d;
using Mat34 = Eigen::Matrix<double, 3, 4>;

using SE3 = Sophus::SE3d;

using namespace mono_slam;

namespace math_utils {

inline int uniform_random_int(const int low, const int high) {
  const unsigned seed =
      std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_int_distribution<int> distribution(low, high);
  auto dice_once = std::bind(distribution, generator);
  return dice_once();
}

inline double degree2radian(const double degree) {
  return degree * EIGEN_PI / 180.0;
}

inline double radian2degree(const double radian) {
  return radian * 180.0 / EIGEN_PI;
}

inline Mat34 kRt2mat(const Mat33& K, const Mat33& R, const Vec3& t) {
  Mat34 Rt;
  Rt.leftCols(3) = R;
  Rt.rightCols(1) = t;
  return K * Rt;
}

template <typename T>
inline T get_median(std::vector<T>& data_vec) {
  CHECK_EQ(data_vec.empty(), false);
  typename std::vector<T>::iterator it = std::next(
      data_vec.begin(), std::floor(static_cast<int>(data_vec.size()) / 2));
  std::nth_element(data_vec.begin(), it, data_vec.end());
  return *it;
}

inline SE3 arma_to_SE3(const arma::rowvec& pose) {
  SE3 pose_SE3;
  return pose_SE3;
}

inline Eigen::Affine3f SE3_to_affine(const SE3& pose) {
  Eigen::Affine3f T;
  // Affine part = linear part + scale.
  T.linear() = pose.rotationMatrix().cast<float>();
  T.translation() = pose.translation().cast<float>();
  return T;
}

}  // namespace math_utils

#endif  // MONO_SLAM_UTILS_MATH_UTILS_H_