#ifndef MONO_SLAM_CAMERA_H_
#define MONO_SLAM_CAMERA_H_

#include "mono_slam/common_include.h"

namespace mono_slam {

class Camera {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = uptr<Camera>;

  // Empty constructor used only when setting camera parameters.
  Camera();

  Camera(const SE3& T_c_w);

  void init(const double fx, const double fy, const double cx, const double cy,
            const Vec4& dist_coeffs);

  // Getters.
  inline const SE3& pose() const { return T_c_w_; }
  inline const Vec3& pos() const { return T_c_w_.inverse().translation(); }
  inline const Mat33& K() const { return K_; }
  inline const Vec4& distCoeffs() const { return dist_coeffs_; }

  // Setters.
  void setPose(const SE3& T_c_w);
  void setPos(const Vec3& pos);

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

  inline Vec3 pixel2bear(const Vec2& pt) const {
    // TODO(bayes)
    return Vec3{};
  }

  static inline Mat34 to_cam_mat(const Mat33& K, const Mat33& R,
                                 const Vec3& t) {
    Mat34 Rt;
    Rt.leftCols(3) = R;
    Rt.rightCols(1) = t;
    return K * Rt;
  }

  // Camera center in world frame.
  inline Vec3 getCameraCenter() const {
    return -T_c_w_.rotationMatrix().transpose() * T_c_w_.translation();
  }

  inline double getDistanceToCenter(const Vec3& p_w) const {
    return (p_w - this->getCameraCenter()).norm();
  }

  inline Vec3 getUnitBearVec(const Vec3& p_w) const {
    const Vec3 unit_bear_vec = p_w - this->getCameraCenter();
    return unit_bear_vec / unit_bear_vec.norm();
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