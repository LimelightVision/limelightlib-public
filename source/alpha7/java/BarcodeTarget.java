// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

/** A barcode/QR code target. */
public class BarcodeTarget extends LimelightTarget {
  /** Barcode family (e.g. "QRCODE") */
  public String family = "";

  /** Decoded barcode data string. */
  public String data = "";
}
