// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

/** The robot-orientation state the camera is using for MegaTag2. */
public class RobotOrientationData {
  /** Active IMU mode ({@link IMUMode#UNKNOWN} before the first frame). */
  public IMUMode imuMode = IMUMode.UNKNOWN;

  /** IMU assist alpha. */
  public double alpha;

  /** Last interpolated robot yaw used for botpose, in degrees. */
  public double interpolatedYawDegrees;
}
