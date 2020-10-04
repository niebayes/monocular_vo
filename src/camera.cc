#include "mono_slam/camera.h"

namespace mono_slam {

Camera::Camera() {}

Camera::Camera(const SE3& T_c_w) : T_c_w_(T_c_w) {}

void Camera::init(const double fx, const double fy, const double cx,
                  const double cy, const Vec4& dist_coeffs) {
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
  K_ = (Mat33() << fx, 0., cx, 0., fy, cy, 0., 0., 1.).finished();
  dist_coeffs_ = dist_coeffs;
}

// Setters.
void Camera::setPose(const SE3& T_c_w) { T_c_w_ = T_c_w; }
void Camera::setPos(const Vec3& pos) { T_c_w_.translation() = pos; }

}  // namespace mono_slam
