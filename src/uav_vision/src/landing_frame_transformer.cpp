#include "uav_vision/landing_frame_transformer.hpp"

#include <cmath>
#include <stdexcept>

namespace uav_vision {

std::array<double, 9> cameraToBodyFromRpyDegrees(
    double roll_degrees,
    double pitch_degrees,
    double yaw_degrees) {
  if (!std::isfinite(roll_degrees) || !std::isfinite(pitch_degrees) ||
      !std::isfinite(yaw_degrees)) {
    throw std::invalid_argument("camera-to-body RPY must be finite");
  }
  constexpr double kDegreesToRadians = 0.017453292519943295;
  const double roll = roll_degrees * kDegreesToRadians;
  const double pitch = pitch_degrees * kDegreesToRadians;
  const double yaw = yaw_degrees * kDegreesToRadians;
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  // Intrinsic camera-axis roll/pitch/yaw, composed as Rz(yaw) * Ry(pitch) *
  // Rx(roll). The resulting matrix maps a camera ray into the body frame.
  return {{
      cy * cp,
      cy * sp * sr - sy * cr,
      cy * sp * cr + sy * sr,
      sy * cp,
      sy * sp * sr + cy * cr,
      sy * sp * cr - cy * sr,
      -sp,
      cp * sr,
      cp * cr,
  }};
}

LandingFrameTransformer::LandingFrameTransformer(
    const std::array<double, 9>& camera_to_body)
    : camera_to_body_(camera_to_body) {
  for (const double value : camera_to_body_) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("camera_to_body must contain finite values");
    }
  }
}

PlanarOffset LandingFrameTransformer::cameraToLeveledImage(
    double camera_x,
    double camera_y,
    double focal_length_px,
    const AttitudeQuaternion& world_from_body) const {
  PlanarOffset result;
  if (!std::isfinite(camera_x) || !std::isfinite(camera_y) ||
      !std::isfinite(focal_length_px) || focal_length_px <= 0.0) {
    return result;
  }

  const double norm = std::sqrt(
      world_from_body.w * world_from_body.w +
      world_from_body.x * world_from_body.x +
      world_from_body.y * world_from_body.y +
      world_from_body.z * world_from_body.z);
  if (!std::isfinite(norm) || norm < 1e-9) {
    return result;
  }

  const double w = world_from_body.w / norm;
  const double x = world_from_body.x / norm;
  const double y = world_from_body.y / norm;
  const double z = world_from_body.z / norm;

  const double camera_ray_x = camera_x / focal_length_px;
  const double camera_ray_y = camera_y / focal_length_px;
  constexpr double kCameraRayZ = 1.0;
  const double body_x =
      camera_to_body_[0] * camera_ray_x +
      camera_to_body_[1] * camera_ray_y +
      camera_to_body_[2] * kCameraRayZ;
  const double body_y =
      camera_to_body_[3] * camera_ray_x +
      camera_to_body_[4] * camera_ray_y +
      camera_to_body_[5] * kCameraRayZ;
  const double body_z =
      camera_to_body_[6] * camera_ray_x +
      camera_to_body_[7] * camera_ray_y +
      camera_to_body_[8] * kCameraRayZ;

  const double r00 = 1.0 - 2.0 * (y * y + z * z);
  const double r01 = 2.0 * (x * y - w * z);
  const double r02 = 2.0 * (x * z + w * y);
  const double r10 = 2.0 * (x * y + w * z);
  const double r11 = 1.0 - 2.0 * (x * x + z * z);
  const double r12 = 2.0 * (y * z - w * x);

  const double r20 = 2.0 * (x * z - w * y);
  const double r21 = 2.0 * (y * z + w * x);
  const double r22 = 1.0 - 2.0 * (x * x + y * y);

  // Express the ray in a gravity-leveled body frame that keeps the current
  // aircraft yaw. R_level_body = Rz(-yaw) * R_world_body, so pure yaw is an
  // identity operation while roll/pitch image motion is removed.
  const double yaw = std::atan2(r10, r00);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  const double level_body_x =
      (cy * r00 + sy * r10) * body_x +
      (cy * r01 + sy * r11) * body_y +
      (cy * r02 + sy * r12) * body_z;
  const double level_body_y =
      (-sy * r00 + cy * r10) * body_x +
      (-sy * r01 + cy * r11) * body_y +
      (-sy * r02 + cy * r12) * body_z;
  const double level_body_z =
      r20 * body_x + r21 * body_y + r22 * body_z;

  // Return to the same image-axis convention as the physical camera so dx/dy
  // remain directly comparable to the raw signed pixel offsets.
  const double level_camera_x =
      camera_to_body_[0] * level_body_x +
      camera_to_body_[3] * level_body_y +
      camera_to_body_[6] * level_body_z;
  const double level_camera_y =
      camera_to_body_[1] * level_body_x +
      camera_to_body_[4] * level_body_y +
      camera_to_body_[7] * level_body_z;
  const double level_camera_z =
      camera_to_body_[2] * level_body_x +
      camera_to_body_[5] * level_body_y +
      camera_to_body_[8] * level_body_z;
  if (!std::isfinite(level_camera_z) ||
      std::abs(level_camera_z) < 1e-6) {
    return result;
  }

  result.x = focal_length_px * level_camera_x / level_camera_z;
  result.y = focal_length_px * level_camera_y / level_camera_z;
  result.valid = std::isfinite(result.x) && std::isfinite(result.y);
  return result;
}

}  // namespace uav_vision
