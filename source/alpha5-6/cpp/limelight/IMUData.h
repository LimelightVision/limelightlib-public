// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <vector>

#include "wpi/units/angle.hpp"
#include "wpi/units/angular_velocity.hpp"

namespace limelight {

/**
 * The state of the camera's internal IMU.
 */
struct IMUData {
  /** Orientation quaternion [w, x, y, z] */
  std::vector<double> quaternion{1, 0, 0, 0};
  /** Final fused robot yaw in degrees. MegaTag2 uses this value. Includes
   *  yawOffset */
  wpi::units::degree_t robotYaw{0.0};
  /** Yaw offset in degrees currently applied to the internal IMU */
  wpi::units::degree_t yawOffset{0.0};
  /** Internal IMU roll in degrees */
  wpi::units::degree_t roll{0.0};
  /** Internal IMU pitch in degrees */
  wpi::units::degree_t pitch{0.0};
  /** Raw internal IMU yaw in degrees, before yawOffset. See
   *  robotYaw */
  wpi::units::degree_t yaw{0.0};
  /** Gyro angular velocity about X in degrees per second */
  wpi::units::degrees_per_second_t gyroX{0.0};
  /** Gyro angular velocity about Y in degrees per second */
  wpi::units::degrees_per_second_t gyroY{0.0};
  /** Gyro angular velocity about Z in degrees per second */
  wpi::units::degrees_per_second_t gyroZ{0.0};
  /** Accelerometer X (forward) axis */
  double accelX = 0;
  /** Accelerometer Y (left) axis */
  double accelY = 0;
  /** Accelerometer Z (up) axis */
  double accelZ = 0;
};

}  // namespace limelight
