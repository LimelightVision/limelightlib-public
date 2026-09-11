// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;

/** A robot pose estimate with the metadata needed for pose-estimator fusion. */
public class PoseEstimate {
  public Pose2d pose = new Pose2d();

  /**
   * Latency-compensated capture timestamp in the local NetworkTables timebase. Pass this value
   * directly to {@code addVisionMeasurement} in WPILib.
   */
  public double timestampSeconds;

  /** Total latency (capture + targeting) in milliseconds. */
  public double latencyMillis;

  /** Camera-reported botpose tag count, telemetry only. */
  public int reportedTagCount;

  /** Number of fielded tags. Used for validity, filtering, and standard deviation scaling */
  public int fieldedTagCount;

  /** Max distance between contributing tags in meters. */
  public double tagSpanMeters;

  /** Average distance to contributing tags in meters. */
  public double avgTagDistanceMeters;

  /** Average area of contributing tags (percentage of image). */
  public double avgTagAreaPercent;

  /**
   * Fusion-ready standard deviations [x meters, y meters, theta radians]. The {@link
   * PoseEstimateConfig} of this camera computes them from tag distance and tag count. Pass this
   * value as the third argument to {@code addVisionMeasurement}.
   */
  public Vector<N3> stdDevs =
      VecBuilder.fill(
          PoseEstimateConfig.UNTRUSTED, PoseEstimateConfig.UNTRUSTED, PoseEstimateConfig.UNTRUSTED);

  /**
   * Standard deviations [x, y, z, roll, pitch, yaw] reported by the camera. The camera averages
   * them over a long window. Use them for telemetry only. Do not give them to a pose estimator. Use
   * {@link #stdDevs} instead.
   */
  public double[] reportedStdDevs = Limelight.EMPTY;

  /** The fiducials visible in this frame. */
  public FiducialTarget[] rawFiducials = new FiducialTarget[0];

  /** Which pose estimate output this is (origin + algorithm). */
  public PoseEstimateType type = PoseEstimateType.MT1_WPIBLUE;

  /**
   * The frame index of the source camera. Estimates from the same camera with the same index came
   * from one frame. Do not fuse more than one pose type from one frame. The index can reset when
   * the camera restarts.
   */
  public long frameIndex = -1;

  /**
   * OR of the {@link PoseEstimateConfig} REJECT_* flags for every check this estimate failed. 0 =
   * accepted. Decode with {@link PoseEstimateConfig#describeRejection}. The flags reflect the
   * filter configured for the algorithm of this estimate.
   */
  public int rejectionFlags;

  /**
   * Returns the visible fiducials that contributed to pose estimation.
   *
   * @return The visible fiducials that contributed to pose estimation. These have {@link
   *     FiducialTarget#fielded} set
   */
  public FiducialTarget[] getFieldedFiducials() {
    int count = 0;
    for (FiducialTarget fiducial : rawFiducials) {
      if (fiducial.fielded) {
        count++;
      }
    }
    FiducialTarget[] out = new FiducialTarget[count];
    int i = 0;
    for (FiducialTarget fiducial : rawFiducials) {
      if (fiducial.fielded) {
        out[i++] = fiducial;
      }
    }
    return out;
  }

  /**
   * Returns true if this estimate was produced by the MegaTag2 algorithm.
   *
   * @return True if this estimate was produced by the MegaTag2 algorithm
   */
  public boolean isMT2() {
    return Limelight.usesMT2(type);
  }

  /**
   * Returns true if at least one fielded tag produced this estimate and no check rejected it.
   *
   * @return True if at least one fielded tag produced this estimate and no check rejected it. This
   *     does not show current camera health. It does not show whether the frame was already read.
   */
  public boolean isValid() {
    return fieldedTagCount > 0 && rejectionFlags == 0;
  }

  @Override
  public String toString() {
    return String.format(
        "PoseEstimate(%s x=%.2f y=%.2f deg=%.1f fieldedTags=%d reportedTags=%d "
            + "avgDist=%.2f ts=%.3f%s)",
        type,
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees(),
        fieldedTagCount,
        reportedTagCount,
        avgTagDistanceMeters,
        timestampSeconds,
        rejectionFlags != 0
            ? " REJECTED:" + PoseEstimateConfig.describeRejection(rejectionFlags)
            : "");
  }
}
