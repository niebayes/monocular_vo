#ifndef MONO_SLAM_MATH_UTILS_H_
#define MONO_SLAM_MATH_UTILS_H_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <vector>

#include "glog/logging.h"
#include "mono_slam/config.h"  // FIXME It this okay?

namespace math_utils {

int uniform_random_int(const int low, const int high) {
  const unsigned seed =
      std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_int_distribution<int> distribution(low, high);
  auto dice_once = std::bind(distribution, generator);
  return dice_once();
}

double degree2radian(const double degree) { return degree * EIGEN_PI / 180.0; }

double radian2degree(const double radian) { return radian * 180.0 / EIGEN_PI; }

// FIXME Should this function "sort" data vector in place?
template <typename T>
T get_median(std::vector<T>& data_vec) {
  CHECK_EQ(data_vec.empty(), false);
  typename std::vector<T>::iterator it = std::next(
      data_vec.begin(), std::floor(static_cast<int>(data_vec.size()) / 2));
  std::nth_element(data_vec.begin(), it, data_vec.end());
  return *it;
}

// Estimate the image pyramid level of the feature in the given frame
// corresponding to this map point.
inline int predictLevel(const double dist) const {
  const double level =
      std::log(max_scale_invariance_ / dist) / std::log(Config::scale_factor());
  return std::round(std::clamp(level, 0, Config::num_scale_levels()));
}

}  // namespace math_utils

#endif  // MONO_SLAM_MATH_UTILS_H_