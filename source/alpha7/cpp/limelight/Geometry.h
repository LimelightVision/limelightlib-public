// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <cstddef>
#include <span>

#include "wpi/math/geometry/Pose2d.hpp"
#include "wpi/math/geometry/Pose3d.hpp"
#include "wpi/math/geometry/Rotation2d.hpp"
#include "wpi/math/geometry/Rotation3d.hpp"
#include "wpi/math/geometry/Translation2d.hpp"
#include "wpi/math/geometry/Translation3d.hpp"
#include "wpi/units/angle.hpp"
#include "wpi/units/length.hpp"
namespace limelight {

/** Length of every pose array: [x, y, z, roll, pitch, yaw]. The camera always
 *  sends six values, all zero when there is no pose. Pose and standard
 *  deviation arrays default to six zeros, so index access is safe before the
 *  first frame. */
inline constexpr size_t kPoseArrayLength = 6;

/**
 * Converts a pose array to a Pose3d. The array has 6 values: [x, y, z, roll,
 * pitch, yaw]. Units are meters and degrees.
 * @return The pose. Returns an empty Pose3d if the array is not valid
 */
inline wpi::math::Pose3d ToPose3D(std::span<const double> inData) {
  if (inData.size() < 6) {
    return wpi::math::Pose3d{};
  }
  return wpi::math::Pose3d{
      wpi::math::Translation3d{wpi::units::meter_t{inData[0]},
                               wpi::units::meter_t{inData[1]},
                               wpi::units::meter_t{inData[2]}},
      wpi::math::Rotation3d{wpi::units::degree_t{inData[3]},
                            wpi::units::degree_t{inData[4]},
                            wpi::units::degree_t{inData[5]}}};
}

/**
 * Converts a pose array to a Pose2d. The array has 6 values: [x, y, z, roll,
 * pitch, yaw]. Units are meters and degrees. Uses only the x, y, and yaw
 * values.
 * @return The pose. Returns an empty Pose2d if the array is not valid
 */
inline wpi::math::Pose2d ToPose2D(std::span<const double> inData) {
  if (inData.size() < 6) {
    return wpi::math::Pose2d{};
  }
  return wpi::math::Pose2d{
      wpi::math::Translation2d{wpi::units::meter_t{inData[0]},
                               wpi::units::meter_t{inData[1]}},
      wpi::math::Rotation2d{wpi::units::degree_t{inData[5]}}};
}

}  // namespace limelight
