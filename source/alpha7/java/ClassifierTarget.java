// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

/** A neural classifier result. */
public class ClassifierTarget {
  /** Class index from the class list. */
  public int classId = -1;

  /** Human-readable class name. */
  public String className = "";

  /** Classification confidence (0-1). */
  public double confidence;
}
