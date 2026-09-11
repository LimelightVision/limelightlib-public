// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <cmath>
#include <span>
#include <string>
#include <vector>

#include "limelight/Geometry.h"
#include "limelight/LimelightTarget.h"
#include "wpi/math/geometry/Pose3d.hpp"
namespace limelight {

/**
 * An AprilTag/fiducial target.
 */
struct FiducialTarget : public LimelightTarget {
  /** Fiducial/AprilTag ID */
  int fiducialId = 0;
  /** Fiducial family (e.g. "36h11") */
  std::string family;
  /** Target skew values */
  std::vector<double> skew;
  /** Pose ambiguity. Lower is better. Values above 0.7 are usually not
   *  reliable */
  double ambiguity = 0;
  /** True if this tag contributed to pose estimation. The tag matched the
   *  field map and the ID filters did not exclude it. */
  bool fielded = false;

  /** Camera pose in target space [x, y, z, roll, pitch, yaw] (meters,
   *  degrees) */
  std::vector<double> cameraPoseTargetSpace =
      std::vector<double>(kPoseArrayLength, 0.0);
  /** Target pose in camera space */
  std::vector<double> targetPoseCameraSpace =
      std::vector<double>(kPoseArrayLength, 0.0);
  /** Target pose in robot space */
  std::vector<double> targetPoseRobotSpace =
      std::vector<double>(kPoseArrayLength, 0.0);
  /** Robot pose in target space */
  std::vector<double> robotPoseTargetSpace =
      std::vector<double>(kPoseArrayLength, 0.0);
  /** Robot pose in field space (MegaTag1, this tag only) */
  std::vector<double> robotPoseFieldSpace =
      std::vector<double>(kPoseArrayLength, 0.0);
  /** Robot pose in field space (MegaTag2, this tag only) */
  std::vector<double> robotPoseFieldSpaceMT2 =
      std::vector<double>(kPoseArrayLength, 0.0);

  /** @return 3D distance from the camera to this tag in meters */
  double GetDistanceToCamera() const {
    return Distance3d(targetPoseCameraSpace);
  }

  /** @return 3D distance from the robot center to this tag in meters */
  double GetDistanceToRobot() const { return Distance3d(targetPoseRobotSpace); }

  /** @return Camera pose in target space as a Pose3d */
  wpi::math::Pose3d GetCameraPose_TargetSpace() const {
    return ToPose3D(cameraPoseTargetSpace);
  }

  /** @return Target pose in camera space as a Pose3d */
  wpi::math::Pose3d GetTargetPose_CameraSpace() const {
    return ToPose3D(targetPoseCameraSpace);
  }

  /** @return Target pose in robot space as a Pose3d */
  wpi::math::Pose3d GetTargetPose_RobotSpace() const {
    return ToPose3D(targetPoseRobotSpace);
  }

  /** @return Robot pose in target space as a Pose3d */
  wpi::math::Pose3d GetRobotPose_TargetSpace() const {
    return ToPose3D(robotPoseTargetSpace);
  }

  /** @return MegaTag1 robot pose in field space from this tag only, as a Pose3d
   */
  wpi::math::Pose3d GetRobotPose_FieldSpace() const {
    return ToPose3D(robotPoseFieldSpace);
  }

  /** @return MegaTag2 robot pose in field space from this tag only, as a Pose3d
   */
  wpi::math::Pose3d GetRobotPose_FieldSpace_MT2() const {
    return ToPose3D(robotPoseFieldSpaceMT2);
  }

 private:
  static double Distance3d(std::span<const double> pose) {
    if (pose.size() < 3) {
      return 0;
    }
    return std::sqrt(pose[0] * pose[0] + pose[1] * pose[1] + pose[2] * pose[2]);
  }
};

}  // namespace limelight
