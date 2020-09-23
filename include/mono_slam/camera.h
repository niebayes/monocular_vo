#ifndef MONO_SLAM_CAMERA_H_
#define MONO_SLAM_CAMERA_H_

#include "mono_slam/common_include.h"

namespace mono_slam {

class Camera {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = uptr<Camera>;

  Camera() {}

  Camera(const SE3& T_c_w) : T_c_w_(T_c_w) {}

  Camera(const double fx, const double fy, const double cx, const double cy,
         const Vec4& dist_coeffs) {
    fx_ = fx;
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;
    K_ = (Mat33() << fx, 0., cx, 0., fy, cy, 0., 0., 1.).finished();
    dist_coeffs_ = dist_coeffs;
  }

  // Set pose.
  void SetPose(const SE3& T_c_w);

  // Getters.
  inline SE3 pose() const { return T_c_w_; }
  inline Mat33 K() const { return K_; }
  inline Vec4 DistCoeffs() const { return dist_coeffs_; }

  // Transformation utilites.
  // Transform map point in world frame to camera frame.
  inline Vec3 world2camera(const Vec3& p_w) const { return T_c_w_ * p_w; }
  // Transform map point in camera frame to world frame.
  inline Vec3 camera2world(const Vec3& p_c) const {
    return T_c_w_.inverse() * p_c;
  }
  // Transform map point wrt. this camera to image point expressed in pixels
  inline Vec2 camera2pixel(const Vec3& p_c) const {
    return Vec2{fx_ * p_c(0) / p_c(2) + cx_, fy_ * p_c(1) / p_c(2) + cy_};
  }
  // Transform image point expressed in pixels to map point wrt. this
  // camera.
  inline Vec3 pixel2camera(const Vec2& pt, const double depth) const {
    CHECK_GE(depth, 0.);
    return depth * Vec3{(pt.x() - cx_) / fx_, (pt.y() - cy_) / fy_, 1.0};
  }
  // Transform map point in world frame to image point expressed in pixels wrt.
  // this camera.
  inline Vec2 world2pixel(const Vec3& p_w) const {
    return camera2pixel(world2camera(p_w));
  }
  // Transform image point expressed in pixels wrt. this camera to map point in
  // world frame.
  inline Vec3 pixel2world(const Vec2& pt, const double depth) const {
    CHECK_GE(depth, 0.);
    return camera2world(pixel2camera(pt, depth));
  }

 private:
  SE3 T_c_w_;
  // Camera intrinsics.
  static double fx_;
  static double fy_;
  static double cx_;
  static double cy_;
  static Mat33 K_;
  static Vec4 dist_coeffs_;
};

}  // namespace mono_slam

#endif  // MONO_SLAM_CAMERA_H_