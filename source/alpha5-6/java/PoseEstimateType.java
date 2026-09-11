// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

/**
 * The robot pose estimate outputs. Names use the form {algorithm}_{origin}.
 *
 * <p>MegaTag1 (MT1) computes the full robot pose from tag geometry only. MegaTag2 (MT2) also uses
 * the robot orientation from {@link Limelight#setRobotOrientation} or {@link
 * Limelight#setSharedRobotOrientation}. MT2 is usually more stable. MT2 requires you to publish the
 * orientation every loop.
 */
public enum PoseEstimateType {
  /** MegaTag1, WPILib blue-alliance-corner origin (botpose_wpiblue). */
  MT1_WPIBLUE,
  /** MegaTag1, WPILib red-alliance-corner origin (botpose_wpired). */
  MT1_WPIRED,
  /** MegaTag2, WPILib blue-alliance-corner origin (botpose_orb_wpiblue). */
  MT2_WPIBLUE,
  /** MegaTag2, WPILib red-alliance-corner origin (botpose_orb_wpired). */
  MT2_WPIRED
}
