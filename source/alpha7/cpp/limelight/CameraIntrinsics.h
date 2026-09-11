// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <vector>

#include "wpi/units/angle.hpp"

namespace limelight {

/**
 * The active camera intrinsics, scaled to the current processing resolution.
 */
struct CameraIntrinsics {
  /** True when the camera uses a user-uploaded calibration file. False when
   *  the camera uses a default calibration */
  bool customCalibration = false;
  /** Horizontal field of view in degrees */
  wpi::units::degree_t hfov{0.0};
  /** Vertical field of view in degrees */
  wpi::units::degree_t vfov{0.0};
  /** Processing-resolution width in pixels. The camera matrix is scaled to
   *  this width */
  double resolutionWidthPixels = 0;
  /** Processing-resolution height in pixels */
  double resolutionHeightPixels = 0;
  /** Row-major 3x3 camera matrix [fx, 0, cx, 0, fy, cy, 0, 0, 1], scaled to
   *  the processing resolution. Empty until the camera publishes */
  std::vector<double> cameraMatrix;
  /** OpenCV distortion coefficients. Empty until the camera publishes */
  std::vector<double> distortionCoefficients;
};

}  // namespace limelight
