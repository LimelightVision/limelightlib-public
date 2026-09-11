// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

import org.wpilib.math.geometry.Pose3d;

/** A color/retroreflective target. */
public class RetroTarget extends LimelightTarget {
  /** Camera pose in target space [x, y, z, roll, pitch, yaw] (meters, degrees). */
  public double[] cameraPoseTargetSpace = new double[Limelight.POSE_ARRAY_LENGTH];

  /** Target pose in camera space. */
  public double[] targetPoseCameraSpace = new double[Limelight.POSE_ARRAY_LENGTH];

  /** Target pose in robot space. */
  public double[] targetPoseRobotSpace = new double[Limelight.POSE_ARRAY_LENGTH];

  /** Robot pose in target space. */
  public double[] robotPoseTargetSpace = new double[Limelight.POSE_ARRAY_LENGTH];

  /** Robot pose in field space. */
  public double[] robotPoseFieldSpace = new double[Limelight.POSE_ARRAY_LENGTH];

  /**
   * Returns camera pose in target space as a Pose3d.
   *
   * @return Camera pose in target space as a Pose3d
   */
  public Pose3d getCameraPose_TargetSpace() {
    return Limelight.toPose3D(cameraPoseTargetSpace);
  }

  /**
   * Returns target pose in camera space as a Pose3d.
   *
   * @return Target pose in camera space as a Pose3d
   */
  public Pose3d getTargetPose_CameraSpace() {
    return Limelight.toPose3D(targetPoseCameraSpace);
  }

  /**
   * Returns target pose in robot space as a Pose3d.
   *
   * @return Target pose in robot space as a Pose3d
   */
  public Pose3d getTargetPose_RobotSpace() {
    return Limelight.toPose3D(targetPoseRobotSpace);
  }

  /**
   * Returns robot pose in target space as a Pose3d.
   *
   * @return Robot pose in target space as a Pose3d
   */
  public Pose3d getRobotPose_TargetSpace() {
    return Limelight.toPose3D(robotPoseTargetSpace);
  }

  /**
   * Returns robot pose in field space as a Pose3d.
   *
   * @return Robot pose in field space as a Pose3d
   */
  public Pose3d getRobotPose_FieldSpace() {
    return Limelight.toPose3D(robotPoseFieldSpace);
  }
}
