// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

/** An OCR (text recognition) result. */
public class OCRTarget {
  public boolean valid;
  public int regionId = -1;

  /** Recognized text. */
  public String text = "";

  public double confidence;
  public double numericValue;
  public boolean hasNumericValue;
  public int digitCount;
  public String unit = "";
  public boolean isCounter;
  public double ratePerMin;
  public int alarmState;
  public String alarmString = "";

  /** Bounding box [x, y, width, height] in pixels. */
  public double[] boundingBox = Limelight.EMPTY;

  public double processingTimeMs;
}
