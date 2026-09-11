// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <string>

#include "limelight/LimelightTarget.h"
namespace limelight {

/**
 * A barcode/QR code target.
 */
struct BarcodeTarget : public LimelightTarget {
  /** Barcode family (e.g. "QRCODE") */
  std::string family;
  /** Decoded barcode data string */
  std::string data;
};

}  // namespace limelight
