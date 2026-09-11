// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <cstdint>

#include "wpi/units/time.hpp"

namespace limelight {

/**
 * Rewind buffer state.
 */
struct RewindData {
  bool enabled = false;
  wpi::units::second_t stored{0.0};
  int64_t frameCount = 0;
  double bufferUsage = 0;
  bool flushing = false;
  /** Main-thread latency penalty in microseconds */
  wpi::units::microsecond_t latencyPenalty{0.0};
};

}  // namespace limelight
