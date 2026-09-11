// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

/** The state of the camera's internal IMU. */
public class IMUData {
  /** Orientation quaternion [w, x, y, z]. */
  public double[] quaternion = {1, 0, 0, 0};

  /** Final fused robot yaw in degrees, the value MegaTag2 uses (includes yawOffsetDegrees). */
  public double robotYawDegrees;

  /** Yaw offset in degrees currently applied to the internal IMU. */
  public double yawOffsetDegrees;

  /** Internal IMU roll in degrees. */
  public double rollDegrees;

  /** Internal IMU pitch in degrees. */
  public double pitchDegrees;

  /** Raw internal IMU yaw in degrees (before yawOffsetDegrees, see {@link #robotYawDegrees}). */
  public double yawDegrees;

  /** Gyro angular velocity about X in degrees per second. */
  public double gyroXDegreesPerSecond;

  /** Gyro angular velocity about Y in degrees per second. */
  public double gyroYDegreesPerSecond;

  /** Gyro angular velocity about Z in degrees per second. */
  public double gyroZDegreesPerSecond;

  /** Accelerometer X (forward) axis. */
  public double accelX;

  /** Accelerometer Y (left) axis. */
  public double accelY;

  /** Accelerometer Z (up) axis. */
  public double accelZ;
}
