#pragma once

#include <array>

namespace uav_vision {

struct AttitudeQuaternion {
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct PlanarOffset {
  bool valid = false;
  double x = 0.0;
  double y = 0.0;
};

std::array<double, 9> cameraToBodyFromRpyDegrees(
    double roll_degrees,
    double pitch_degrees,
    double yaw_degrees);

class LandingFrameTransformer {
 public:
  explicit LandingFrameTransformer(
      const std::array<double, 9>& camera_to_body);

  PlanarOffset cameraToLeveledImage(
      double camera_x,
      double camera_y,
      double focal_length_px,
      const AttitudeQuaternion& world_from_body) const;

 private:
  std::array<double, 9> camera_to_body_;
};

}  // namespace uav_vision
