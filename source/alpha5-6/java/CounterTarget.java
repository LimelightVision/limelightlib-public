// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

/** A per-class object counter result from the neural counter pipeline. */
public class CounterTarget {
  public int classId = -1;
  public String className = "";
  public int count;
}
