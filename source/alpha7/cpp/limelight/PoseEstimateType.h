// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <cstddef>
#include <string_view>
namespace limelight {

/**
 * The robot pose estimate outputs. Names use the form {algorithm}_{origin}.
 *
 * MegaTag1 (MT1) computes the full robot pose from tag geometry only.
 * MegaTag2 (MT2) also uses the robot orientation from
 * Limelight::SetRobotOrientation() or Limelight::SetSharedRobotOrientation().
 * MT2 is usually more stable. MT2 requires you to publish the orientation
 * every loop.
 */
enum class PoseEstimateType {
  /** MegaTag1, WPILib blue-alliance-corner origin (botpose_wpiblue) */
  MT1_WPIBLUE,
  /** MegaTag1, WPILib red-alliance-corner origin (botpose_wpired) */
  MT1_WPIRED,
  /** MegaTag2, WPILib blue-alliance-corner origin (botpose_orb_wpiblue) */
  MT2_WPIBLUE,
  /** MegaTag2, WPILib red-alliance-corner origin (botpose_orb_wpired) */
  MT2_WPIRED
};

/** Number of PoseEstimateType values. */
inline constexpr size_t kPoseEstimateTypeCount = 4;

namespace detail {

constexpr bool UsesMT2(PoseEstimateType type) {
  switch (type) {
    case PoseEstimateType::MT1_WPIBLUE:
    case PoseEstimateType::MT1_WPIRED:
      return false;
    case PoseEstimateType::MT2_WPIBLUE:
    case PoseEstimateType::MT2_WPIRED:
    default:
      return true;
  }
}

constexpr bool CenteredOrigin(PoseEstimateType) {
  return false;
}

constexpr std::string_view PoseEstimateTypeName(PoseEstimateType type) {
  switch (type) {
    case PoseEstimateType::MT1_WPIBLUE:
      return "MT1_WPIBLUE";
    case PoseEstimateType::MT1_WPIRED:
      return "MT1_WPIRED";
    case PoseEstimateType::MT2_WPIBLUE:
      return "MT2_WPIBLUE";
    case PoseEstimateType::MT2_WPIRED:
    default:
      return "MT2_WPIRED";
  }
}

}  // namespace detail

}  // namespace limelight
