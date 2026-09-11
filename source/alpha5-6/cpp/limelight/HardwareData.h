// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <cstdint>
#include <string>

#include "wpi/units/power.hpp"
#include "wpi/units/temperature.hpp"

namespace limelight {

/**
 * Camera hardware and system stats.
 */
struct HardwareData {
  /** CPU temperature in degrees Celsius */
  wpi::units::celsius_t cpuTemp{0.0};
  /** CPU usage percentage */
  double cpuUsagePercent = 0;
  /** RAM usage percentage */
  double ramUsagePercent = 0;
  /** Free disk space in MB */
  int64_t diskFreeMB = 0;
  /** Total disk space in MB */
  int64_t diskTotalMB = 0;
  /** Camera sensor ID */
  std::string cameraId;
  /** True if an AI accelerator is present */
  bool accelPresent = false;
  /** AI accelerator type */
  std::string accelType;
  /** AI accelerator temperature in degrees Celsius */
  wpi::units::celsius_t accelTemp{0.0};
  /** AI accelerator power draw in watts */
  wpi::units::watt_t accelPower{0.0};
  /** True if the AI accelerator is throttling */
  bool accelThrottling = false;
};

}  // namespace limelight
