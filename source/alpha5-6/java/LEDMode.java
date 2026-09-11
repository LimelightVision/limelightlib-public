// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

/** LED behavior modes. */
public enum LEDMode {
  /** LED behavior is controlled by the current pipeline. */
  PIPELINE_CONTROL(0),
  /** LEDs forced off. */
  FORCE_OFF(1),
  /** LEDs forced to blink. */
  FORCE_BLINK(2),
  /** LEDs forced on. */
  FORCE_ON(3);

  final int ntValue;

  LEDMode(int ntValue) {
    this.ntValue = ntValue;
  }
}
