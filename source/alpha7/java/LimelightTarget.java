// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

@SuppressWarnings("PMD.AbstractClassWithoutAbstractMethod")
public abstract class LimelightTarget {
  LimelightTarget() {}

  /** Horizontal offset from crosshair to target in degrees. */
  public double txDegrees;

  /** Vertical offset from crosshair to target in degrees. */
  public double tyDegrees;

  /** Horizontal offset from principal pixel to target in degrees. */
  public double txDegreesNoCrosshair;

  /** Vertical offset from principal pixel to target in degrees. */
  public double tyDegreesNoCrosshair;

  /** Target x position in pixels. */
  public double txPixels;

  /** Target y position in pixels. */
  public double tyPixels;

  /** Target area (percentage of image, 0-100). */
  public double targetAreaPercent;

  /** Target corners as [x, y] pixel pairs. Empty unless "send corners" is enabled */
  public double[][] corners = Limelight.EMPTY_CORNERS;
}
