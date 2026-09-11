// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

import org.wpilib.networktables.IntegerPublisher;
import org.wpilib.networktables.NetworkTable;

final class PoseCountsTelemetry {
  private final IntegerPublisher processed;
  private final IntegerPublisher accepted;
  private final IntegerPublisher rejected;
  private final IntegerPublisher[] rejectionCounts =
      new IntegerPublisher[Limelight.REJECTION_FLAGS.length];
  private final long[] rejectionTotals = new long[Limelight.REJECTION_FLAGS.length];
  private long processedTotal;
  private long acceptedTotal;
  private long rejectedTotal;

  PoseCountsTelemetry(NetworkTable telemetryTable) {
    NetworkTable countsTable = telemetryTable.getSubTable("counts");
    processed = countsTable.getIntegerTopic("processed").publish();
    accepted = countsTable.getIntegerTopic("accepted").publish();
    rejected = countsTable.getIntegerTopic("rejected").publish();
    processed.set(0);
    accepted.set(0);
    rejected.set(0);
    for (int i = 0; i < rejectionCounts.length; i++) {
      rejectionCounts[i] = countsTable.getIntegerTopic(Limelight.REJECTION_NAMES[i]).publish();
      rejectionCounts[i].set(0);
    }
  }

  void count(PoseEstimate estimate) {
    processed.set(++processedTotal);
    if (estimate.isValid()) {
      accepted.set(++acceptedTotal);
      return;
    }
    rejected.set(++rejectedTotal);
    for (int i = 0; i < Limelight.REJECTION_FLAGS.length; i++) {
      if ((estimate.rejectionFlags & Limelight.REJECTION_FLAGS[i]) != 0) {
        rejectionCounts[i].set(++rejectionTotals[i]);
      }
    }
  }

  void unpublish() {
    processed.close();
    accepted.close();
    rejected.close();
    for (IntegerPublisher rejectionCount : rejectionCounts) {
      rejectionCount.close();
    }
  }
}
