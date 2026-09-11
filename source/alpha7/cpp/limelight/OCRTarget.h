// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <string>
#include <vector>
namespace limelight {

/**
 * An OCR (text recognition) result.
 */
struct OCRTarget {
  bool valid = false;
  int regionId = -1;
  /** Recognized text */
  std::string text;
  double confidence = 0;
  double numericValue = 0;
  bool hasNumericValue = false;
  int digitCount = 0;
  std::string unit;
  bool isCounter = false;
  double ratePerMin = 0;
  int alarmState = 0;
  std::string alarmString;
  /** Bounding box [x, y, width, height] in pixels */
  std::vector<double> boundingBox;
  double processingTimeMs = 0;
};

}  // namespace limelight
