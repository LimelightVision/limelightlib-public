// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

/** Camera hardware and system stats. */
public class HardwareData {
  /** CPU temperature in degrees Celsius. */
  public double cpuTempCelsius;

  /** CPU usage percentage. */
  public double cpuUsagePercent;

  /** RAM usage percentage. */
  public double ramUsagePercent;

  /** Free disk space in MB. */
  public long diskFreeMB;

  /** Total disk space in MB. */
  public long diskTotalMB;

  /** Camera sensor ID. */
  public String cameraId = "";

  /** True if an AI accelerator is present. */
  public boolean accelPresent;

  /** AI accelerator type. */
  public String accelType = "";

  /** AI accelerator temperature in degrees Celsius. */
  public double accelTempCelsius;

  /** AI accelerator power draw in watts. */
  public double accelPowerWatts;

  /** True if the AI accelerator is throttling. */
  public boolean accelThrottling;
}
