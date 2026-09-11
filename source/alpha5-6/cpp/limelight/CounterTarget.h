// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <string>
namespace limelight {

/**
 * A per-class object counter result from the neural counter pipeline.
 */
struct CounterTarget {
  int classId = -1;
  std::string className;
  int count = 0;
};

}  // namespace limelight
