// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <string>

#include "limelight/LimelightTarget.h"
namespace limelight {

/**
 * A neural detector target.
 */
struct DetectorTarget : public LimelightTarget {
  /** Class index from the class list */
  int classId = -1;
  /** Human-readable class name */
  std::string className;
  /** Detection confidence (0-1) */
  double confidence = 0;
  /** Tracking ID for object counting pipelines. -1 means no tracking */
  int trackId = -1;
};

}  // namespace limelight
