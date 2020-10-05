#include "mono_slam/camera.h"

namespace mono_slam {

Camera::Camera() {}

// Setters.
void Camera::setPose(const SE3& T_c_w) { T_c_w_ = T_c_w; }
void Camera::setPos(const Vec3& pos) { T_c_w_.translation() = pos; }

}  // namespace mono_slam
