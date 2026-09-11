// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

namespace limelight {

/**
 * AprilTag detector downscaling factors. More downscaling improves
 * performance. It can reduce the detection range.
 */
enum class DownscaleOverride {
  /** Use the downscale configured in the current pipeline */
  PIPELINE_CONTROL = 0,
  /** No downscaling */
  X1 = 1,
  /** 1.5x downscale */
  X1_5 = 2,
  /** 2x downscale */
  X2 = 3,
  /** 3x downscale */
  X3 = 4,
  /** 4x downscale */
  X4 = 5
};

}  // namespace limelight
