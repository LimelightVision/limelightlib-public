// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.networktables.DoubleArrayPublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.networktables.StructArrayPublisher;

final class PoseTelemetry {
  private final StructArrayPublisher<Pose2d> accepted;
  private final StructArrayPublisher<Pose2d> rejected;
  private final StringPublisher rejectionReasons;
  private final DoubleArrayPublisher fieldAccepted;
  private final DoubleArrayPublisher fieldRejected;

  PoseTelemetry(
      NetworkTable telemetryTable,
      NetworkTable fieldTable,
      String cameraName,
      PoseEstimateType type) {
    String prefix = type.name() + "/";
    accepted = telemetryTable.getStructArrayTopic(prefix + "accepted", Pose2d.struct).publish();
    rejected = telemetryTable.getStructArrayTopic(prefix + "rejected", Pose2d.struct).publish();
    rejectionReasons = telemetryTable.getStringTopic(prefix + "rejectionReasons").publish();
    fieldAccepted = fieldTable.getDoubleArrayTopic(cameraName + "-" + type.name()).publish();
    fieldRejected =
        fieldTable.getDoubleArrayTopic(cameraName + "-" + type.name() + "-rejected").publish();
  }

  void publish(PoseEstimate estimate) {
    boolean drawRejected =
        estimate.fieldedTagCount > 0
            && estimate.rejectionFlags != 0
            && (estimate.rejectionFlags & PoseEstimateConfig.REJECT_MISSING_POSE) == 0
            && Double.isFinite(estimate.pose.getX())
            && Double.isFinite(estimate.pose.getY())
            && Double.isFinite(estimate.pose.getRotation().getRadians());
    accepted.set(estimate.isValid() ? new Pose2d[] {estimate.pose} : Limelight.EMPTY_POSES);
    rejected.set(drawRejected ? new Pose2d[] {estimate.pose} : Limelight.EMPTY_POSES);
    rejectionReasons.set(PoseEstimateConfig.describeRejection(estimate.rejectionFlags));
    fieldAccepted.set(estimate.isValid() ? Limelight.fieldPose(estimate.pose) : Limelight.EMPTY);
    fieldRejected.set(drawRejected ? Limelight.fieldPose(estimate.pose) : Limelight.EMPTY);
  }

  void clear() {
    accepted.set(Limelight.EMPTY_POSES);
    rejected.set(Limelight.EMPTY_POSES);
    rejectionReasons.set("");
    fieldAccepted.set(Limelight.EMPTY);
    fieldRejected.set(Limelight.EMPTY);
  }

  void unpublish() {
    accepted.close();
    rejected.close();
    rejectionReasons.close();
    fieldAccepted.close();
    fieldRejected.close();
  }
}
