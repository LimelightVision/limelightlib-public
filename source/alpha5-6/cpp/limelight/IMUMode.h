// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

namespace limelight {

/**
 * IMU sources for the robot yaw that MegaTag2 uses. The external orientation
 * comes from Limelight::SetRobotOrientation() or
 * Limelight::SetSharedRobotOrientation(). Only cameras with a built-in IMU use
 * this.
 */
enum class IMUMode {
  /** Use the external orientation directly */
  EXTERNAL = 0,
  /** Uses the external orientation. Continuously seeds the internal IMU with
   *  it */
  EXTERNAL_SEED_INTERNAL = 1,
  /** Use the internal IMU only */
  INTERNAL = 2,
  /** Internal IMU with a complementary filter. The filter converges on the
   *  MegaTag1 yaw */
  INTERNAL_MT1_ASSIST = 3,
  /** Internal IMU with a complementary filter. The filter converges on the
   *  external orientation */
  INTERNAL_EXTERNAL_ASSIST = 4,
  /** The camera reported a mode this library does not know (never valid to
   *  set) */
  UNKNOWN = -1
};

inline IMUMode IMUModeFromNT(int value) {
  switch (value) {
    case 0:
      return IMUMode::EXTERNAL;
    case 1:
      return IMUMode::EXTERNAL_SEED_INTERNAL;
    case 2:
      return IMUMode::INTERNAL;
    case 3:
      return IMUMode::INTERNAL_MT1_ASSIST;
    case 4:
      return IMUMode::INTERNAL_EXTERNAL_ASSIST;
    default:
      return IMUMode::UNKNOWN;
  }
}

}  // namespace limelight
