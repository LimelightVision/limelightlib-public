// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

/**
 * IMU sources for the robot yaw that MegaTag2 uses. The external orientation comes from {@link
 * Limelight#setRobotOrientation} or {@link Limelight#setSharedRobotOrientation}. Only cameras with
 * a built-in IMU use this.
 */
public enum IMUMode {
  /** Use the external orientation directly. */
  EXTERNAL(0),
  /** Uses the external orientation. Continuously seeds the internal IMU with it */
  EXTERNAL_SEED_INTERNAL(1),
  /** Use the internal IMU only. */
  INTERNAL(2),
  /** Internal IMU with a complementary filter. The filter converges on the MegaTag1 yaw */
  INTERNAL_MT1_ASSIST(3),
  /** Internal IMU with a complementary filter converging on the external orientation. */
  INTERNAL_EXTERNAL_ASSIST(4),
  /** The camera reported a mode this library does not know (never valid to set). */
  UNKNOWN(-1);

  final int ntValue;

  IMUMode(int ntValue) {
    this.ntValue = ntValue;
  }

  static IMUMode fromNtValue(int value) {
    for (IMUMode mode : values()) {
      if (mode.ntValue == value && mode != UNKNOWN) {
        return mode;
      }
    }
    return UNKNOWN;
  }
}
