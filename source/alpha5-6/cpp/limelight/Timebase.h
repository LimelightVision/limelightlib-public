// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

namespace limelight {

/** NetworkTables timestamp ticks per second for the WPILib build this library
 *  targets. WPILib 2027 alpha-7 and later use 1e9. Alpha-6 and earlier use
 *  1e6. Divide a NetworkTables timestamp by this value to get seconds. */
inline constexpr double NT_TICKS_PER_SECOND = 1.0e6;

}  // namespace limelight
