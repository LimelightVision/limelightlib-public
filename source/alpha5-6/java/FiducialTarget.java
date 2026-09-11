// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

import org.wpilib.math.geometry.Pose3d;

/** An AprilTag/fiducial target. */
public class FiducialTarget extends LimelightTarget {
  /** Fiducial/AprilTag ID. */
  public int fiducialId;

  /** Fiducial family (e.g. "36h11") */
  public String family = "";

  /** Target skew values. */
  public double[] skew = Limelight.EMPTY;

  /** Pose ambiguity. Lower is better. Values above 0.7 are usually not reliable */
  public double ambiguity;

  /**
   * True if this tag contributed to pose estimation. The tag matched the field map and the ID
   * filters did not exclude it.
   */
  public boolean fielded;

  /** Camera pose in target space [x, y, z, roll, pitch, yaw] (meters, degrees). */
  public double[] cameraPoseTargetSpace = new double[Limelight.POSE_ARRAY_LENGTH];

  /** Target pose in camera space. */
  public double[] targetPoseCameraSpace = new double[Limelight.POSE_ARRAY_LENGTH];

  /** Target pose in robot space. */
  public double[] targetPoseRobotSpace = new double[Limelight.POSE_ARRAY_LENGTH];

  /** Robot pose in target space. */
  public double[] robotPoseTargetSpace = new double[Limelight.POSE_ARRAY_LENGTH];

  /** Robot pose in field space (MegaTag1, this tag only). */
  public double[] robotPoseFieldSpace = new double[Limelight.POSE_ARRAY_LENGTH];

  /** Robot pose in field space (MegaTag2, this tag only). */
  public double[] robotPoseFieldSpaceMT2 = new double[Limelight.POSE_ARRAY_LENGTH];

  /**
   * Returns 3D distance from the camera to this tag in meters.
   *
   * @return 3D distance from the camera to this tag in meters
   */
  public double getDistanceToCamera() {
    return distance3d(targetPoseCameraSpace);
  }

  /**
   * Returns 3D distance from the robot center to this tag in meters.
   *
   * @return 3D distance from the robot center to this tag in meters
   */
  public double getDistanceToRobot() {
    return distance3d(targetPoseRobotSpace);
  }

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
   * Returns megaTag1 robot pose in field space from this tag only, as a Pose3d.
   *
   * @return MegaTag1 robot pose in field space from this tag only, as a Pose3d
   */
  public Pose3d getRobotPose_FieldSpace() {
    return Limelight.toPose3D(robotPoseFieldSpace);
  }

  /**
   * Returns megaTag2 robot pose in field space from this tag only, as a Pose3d.
   *
   * @return MegaTag2 robot pose in field space from this tag only, as a Pose3d
   */
  public Pose3d getRobotPose_FieldSpace_MT2() {
    return Limelight.toPose3D(robotPoseFieldSpaceMT2);
  }

  private static double distance3d(double[] pose) {
    if (pose.length < 3) {
      return 0;
    }
    return Math.sqrt(pose[0] * pose[0] + pose[1] * pose[1] + pose[2] * pose[2]);
  }
}
