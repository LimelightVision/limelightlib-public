// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <string>
namespace limelight {

/**
 * A neural classifier result.
 */
struct ClassifierTarget {
  /** Class index from the class list */
  int classId = -1;
  /** Human-readable class name */
  std::string className;
  /** Classification confidence (0-1) */
  double confidence = 0;
};

}  // namespace limelight
