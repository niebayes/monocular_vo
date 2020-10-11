#ifndef MONO_SLAM_COMMON_INCLUDE_H_
#define MONO_SLAM_COMMON_INCLUDE_H_

// Standard libraries
#include <algorithm>
#include <cassert>
#include <chrono>
#include <forward_list>
#include <iostream>
#include <iterator>
#include <list>
#include <map>
#include <memory>
#include <numeric>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
// multi-threading related
#include <atomic>              // std::atomic
#include <condition_variable>  // std::condition_variable
#include <mutex>               // std::mutex, std::unique_lock
#include <thread>              // std::thread

#include "DBoW3/DBoW3.h"
#include "Eigen/Dense"
#include "Eigen/StdVector"
#define ARMA_ALLOW_FAKE_CLANG
#include "armadillo"
#include "boost/format.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "manif/manif.h"
#include "opencv2/core/eigen.hpp"
#include "opencv2/opencv.hpp"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

// #define DEBUG 1
// #define NO_BA 1
#define NO_LOCAL_MAP 1
#define ENABLE_RESET 1

// using declarations.
using namespace std;
using namespace std::chrono;
using namespace Eigen;
using namespace DBoW3;
using SE3 = Sophus::SE3d;
using SO3 = Sophus::SO3d;

// alias templates for smart pointers.
// TODO(bayes) Use C++20 feature std::atomic_shared_ptr to achieve real thread
// safe.
template <typename T>
using uptr = std::unique_ptr<T>;

template <typename T>
using sptr = std::shared_ptr<T>;

template <typename T>
using wptr = std::weak_ptr<T>;

// typedef for multi-threading
//! Since C++17, the template arguments could be deduced by compiler.
using u_lock = std::unique_lock<std::mutex>;
using lock_g = std::lock_guard<std::mutex>;

// typedefs for eigen
// double matricies
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXX;
typedef Eigen::Matrix<double, 10, 10> Mat1010;
typedef Eigen::Matrix<double, 13, 13> Mat1313;
typedef Eigen::Matrix<double, 8, 10> Mat810;
typedef Eigen::Matrix<double, 8, 3> Mat83;
typedef Eigen::Matrix<double, 6, 6> Mat66;
typedef Eigen::Matrix<double, 6, 3> Mat63;
typedef Eigen::Matrix<double, 5, 3> Mat53;
typedef Eigen::Matrix<double, 4, 3> Mat43;
typedef Eigen::Matrix<double, 4, 2> Mat42;
typedef Eigen::Matrix<double, 3, 3> Mat33;
typedef Eigen::Matrix<double, 2, 2> Mat22;
typedef Eigen::Matrix<double, 8, 8> Mat88;
typedef Eigen::Matrix<double, 7, 7> Mat77;
typedef Eigen::Matrix<double, 4, 9> Mat49;
typedef Eigen::Matrix<double, 8, 9> Mat89;
typedef Eigen::Matrix<double, 9, 4> Mat94;
typedef Eigen::Matrix<double, 9, 8> Mat98;
typedef Eigen::Matrix<double, 8, 1> Mat81;
typedef Eigen::Matrix<double, 1, 8> Mat18;
typedef Eigen::Matrix<double, 9, 1> Mat91;
typedef Eigen::Matrix<double, 1, 9> Mat19;
typedef Eigen::Matrix<double, 8, 4> Mat84;
typedef Eigen::Matrix<double, 4, 8> Mat48;
typedef Eigen::Matrix<double, 4, 4> Mat44;
typedef Eigen::Matrix<double, 3, 6> Mat36;
typedef Eigen::Matrix<double, 3, 4> Mat34;
typedef Eigen::Matrix<double, 14, 14> Mat1414;

// float matricies
typedef Eigen::Matrix<float, 3, 3> Mat33f;
typedef Eigen::Matrix<float, 10, 3> Mat103f;
typedef Eigen::Matrix<float, 2, 2> Mat22f;
typedef Eigen::Matrix<float, 3, 1> Vec3f;
typedef Eigen::Matrix<float, 2, 1> Vec2f;
typedef Eigen::Matrix<float, 6, 1> Vec6f;
typedef Eigen::Matrix<float, 1, 8> Mat18f;
typedef Eigen::Matrix<float, 6, 6> Mat66f;
typedef Eigen::Matrix<float, 8, 8> Mat88f;
typedef Eigen::Matrix<float, 8, 4> Mat84f;
typedef Eigen::Matrix<float, 6, 6> Mat66f;
typedef Eigen::Matrix<float, 4, 4> Mat44f;
typedef Eigen::Matrix<float, 12, 12> Mat1212f;
typedef Eigen::Matrix<float, 13, 13> Mat1313f;
typedef Eigen::Matrix<float, 10, 10> Mat1010f;
typedef Eigen::Matrix<float, 9, 9> Mat99f;
typedef Eigen::Matrix<float, 4, 2> Mat42f;
typedef Eigen::Matrix<float, 6, 2> Mat62f;
typedef Eigen::Matrix<float, 1, 2> Mat12f;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatXXf;
typedef Eigen::Matrix<float, 14, 14> Mat1414f;

// double vectors
typedef Eigen::Matrix<double, 14, 1> Vec14;
typedef Eigen::Matrix<double, 13, 1> Vec13;
typedef Eigen::Matrix<double, 10, 1> Vec10;
typedef Eigen::Matrix<double, 9, 1> Vec9;
typedef Eigen::Matrix<double, 8, 1> Vec8;
typedef Eigen::Matrix<double, 7, 1> Vec7;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::Matrix<double, 5, 1> Vec5;
typedef Eigen::Matrix<double, 4, 1> Vec4;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 2, 1> Vec2;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;

// float vectors
typedef Eigen::Matrix<float, 12, 1> Vec12f;
typedef Eigen::Matrix<float, 8, 1> Vec8f;
typedef Eigen::Matrix<float, 10, 1> Vec10f;
typedef Eigen::Matrix<float, 4, 1> Vec4f;
typedef Eigen::Matrix<float, 12, 1> Vec12f;
typedef Eigen::Matrix<float, 13, 1> Vec13f;
typedef Eigen::Matrix<float, 9, 1> Vec9f;
typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VecXf;
typedef Eigen::Matrix<float, 14, 1> Vec14f;

#endif  // MONO_SLAM_COMMON_INCLUDE_H_