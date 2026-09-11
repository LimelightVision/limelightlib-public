// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

/** Rewind buffer state. */
public class RewindData {
  public boolean enabled;
  public double storedSeconds;
  public long frameCount;
  public double bufferUsage;
  public boolean flushing;

  /** Main-thread latency penalty in microseconds. */
  public int latencyPenaltyMicros;
}
