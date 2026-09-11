// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <vector>

#include "wpi/units/angle.hpp"

namespace limelight {

/**
 * Fields common to all image-space targets.
 */
struct LimelightTarget {
  /** Horizontal offset from crosshair to target in degrees */
  wpi::units::degree_t tx{0.0};
  /** Vertical offset from crosshair to target in degrees */
  wpi::units::degree_t ty{0.0};
  /** Horizontal offset from principal pixel to target in degrees */
  wpi::units::degree_t txNoCrosshair{0.0};
  /** Vertical offset from principal pixel to target in degrees */
  wpi::units::degree_t tyNoCrosshair{0.0};
  /** Target x position in pixels */
  double txPixels = 0;
  /** Target y position in pixels */
  double tyPixels = 0;
  /** Target area (percentage of image, 0-100) */
  double targetAreaPercent = 0;
  /** Target corners as [x, y] pixel pairs. Empty unless "send corners" is
   *  enabled */
  std::vector<std::vector<double>> corners;
};

}  // namespace limelight
