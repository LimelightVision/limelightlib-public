// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

/** The active camera intrinsics, scaled to the current processing resolution. */
public class CameraIntrinsics {
  /**
   * True when the camera uses a user-uploaded calibration file. False when the camera uses a
   * default calibration
   */
  public boolean customCalibration;

  /** Horizontal field of view in degrees. */
  public double hfovDegrees;

  /** Vertical field of view in degrees. */
  public double vfovDegrees;

  /** Processing-resolution width in pixels (what the camera matrix is scaled to). */
  public double resolutionWidthPixels;

  /** Processing-resolution height in pixels. */
  public double resolutionHeightPixels;

  /**
   * Row-major 3x3 camera matrix [fx, 0, cx, 0, fy, cy, 0, 0, 1], scaled to the processing
   * resolution. Empty until the camera publishes
   */
  public double[] cameraMatrix = Limelight.EMPTY;

  /** OpenCV distortion coefficients. Empty until the camera publishes */
  public double[] distortionCoefficients = Limelight.EMPTY;
}
