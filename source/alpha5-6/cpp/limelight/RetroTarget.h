// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <vector>

#include "limelight/Geometry.h"
#include "limelight/LimelightTarget.h"
#include "wpi/math/geometry/Pose3d.hpp"
namespace limelight {

/**
 * A color/retroreflective target.
 */
struct RetroTarget : public LimelightTarget {
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
  /** Robot pose in field space */
  std::vector<double> robotPoseFieldSpace =
      std::vector<double>(kPoseArrayLength, 0.0);

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

  /** @return Robot pose in field space as a Pose3d */
  wpi::math::Pose3d GetRobotPose_FieldSpace() const {
    return ToPose3D(robotPoseFieldSpace);
  }
};

}  // namespace limelight
