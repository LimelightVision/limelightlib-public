// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include "limelight/IMUMode.h"
#include "wpi/units/angle.hpp"

namespace limelight {

/**
 * The robot-orientation state the camera is using for MegaTag2.
 */
struct RobotOrientationData {
  /** Active IMU mode (IMUMode::UNKNOWN before the first frame) */
  IMUMode imuMode = IMUMode::UNKNOWN;
  /** IMU assist alpha */
  double alpha = 0;
  /** Last interpolated robot yaw used for botpose, in degrees */
  wpi::units::degree_t interpolatedYaw{0.0};
};

}  // namespace limelight
