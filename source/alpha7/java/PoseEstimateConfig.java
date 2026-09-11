// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;

/**
 * A reusable configuration object for rejection filters and standard deviation scaling. Start from
 * a factory ({@link #defaultMT1()}, {@link #defaultMT2()}, {@link #noFiltering()}) or from a new
 * instance. Chain the with* methods. Attach the result with {@link
 * Limelight#withPoseEstimateConfig_MT1} or {@link Limelight#withPoseEstimateConfig_MT2}.
 *
 * <p>A new instance accepts every estimate that decoded with a pose and a timestamp and has at
 * least one fielded tag. It uses the same standard deviation model as {@link #defaultMT2()}.
 *
 * <pre>
 * new Limelight("limelight")
 *     .withPoseEstimateConfig_MT1(PoseEstimateConfig.defaultMT1()
 *         .withMinTagCount(2)
 *         .withFieldBounds(17.55, 8.05))
 *     .withPoseEstimateConfig_MT2(PoseEstimateConfig.defaultMT2()
 *         .withMaxAvgTagDistance(4.0)
 *         .withFieldBounds(17.55, 8.05));
 * </pre>
 */
public class PoseEstimateConfig {
  /** Rejection flag: too few contributing tags. */
  public static final int REJECT_TAG_COUNT = 1 << 0;

  /** Rejection flag: single-tag ambiguity above the configured maximum. */
  public static final int REJECT_AMBIGUITY = 1 << 1;

  /** Rejection flag: tag distance above a configured maximum. */
  public static final int REJECT_TAG_DISTANCE = 1 << 2;

  /** Rejection flag: average tag area below the configured minimum. */
  public static final int REJECT_TAG_AREA = 1 << 3;

  /** Rejection flag: pose outside the configured field bounds. */
  public static final int REJECT_FIELD_BOUNDS = 1 << 4;

  /** Rejection flag set by the library: the source envelope had a decode error. */
  public static final int REJECT_DECODE_ERROR = 1 << 5;

  /** Rejection flag set by the library: a value needed for fusion was NaN or infinite. */
  public static final int REJECT_NONFINITE = 1 << 6;

  /**
   * Rejection flag set by the library: no usable capture timestamp. For example, an envelope
   * decoded with {@link Limelight#decode(byte[])} has no receive time
   */
  public static final int REJECT_NO_TIMESTAMP = 1 << 7;

  /**
   * Rejection flag set by the library: the pose array was absent or too short, or it was the
   * all-zero value the camera sends when it has no estimate.
   */
  public static final int REJECT_MISSING_POSE = 1 << 8;

  /**
   * Rejection flag set by the library: tags contributed but the reported tag distance was zero or
   * negative.
   */
  public static final int REJECT_BAD_METADATA = 1 << 9;

  /**
   * Rejection flag set by the library: fiducials were visible but none contributed to pose
   * estimation.
   */
  public static final int REJECT_NO_FIELDED_TAGS = 1 << 11;

  private int minTagCount = 1;
  private double maxSingleTagAmbiguity = 1.0;
  private double maxSingleTagDistance = 0;
  private double maxAvgTagDistance = 0;
  private double minAvgTagArea = 0;
  private double fieldLengthMeters = 0;
  private double fieldWidthMeters = 0;
  private double fieldBoundsMarginMeters = 0.5;

  /** Creates a configuration with the default values. */
  public PoseEstimateConfig() {}

  /**
   * Creates an independent copy of another configuration.
   *
   * @param other The configuration to copy. Null gives the default values
   */
  public PoseEstimateConfig(PoseEstimateConfig other) {
    if (other == null) {
      return;
    }
    this.minTagCount = other.minTagCount;
    this.maxSingleTagAmbiguity = other.maxSingleTagAmbiguity;
    this.maxSingleTagDistance = other.maxSingleTagDistance;
    this.maxAvgTagDistance = other.maxAvgTagDistance;
    this.minAvgTagArea = other.minAvgTagArea;
    this.fieldLengthMeters = other.fieldLengthMeters;
    this.fieldWidthMeters = other.fieldWidthMeters;
    this.fieldBoundsMarginMeters = other.fieldBoundsMarginMeters;
    this.xyStdDev = other.xyStdDev;
    this.thetaStdDev = other.thetaStdDev;
    this.distanceExponent = other.distanceExponent;
    this.minScalingDistance = other.minScalingDistance;
    this.maxScalingDistance = other.maxScalingDistance;
    this.tagCountExponent = other.tagCountExponent;
    this.minXYStdDev = other.minXYStdDev;
    this.maxXYStdDev = other.maxXYStdDev;
    this.minThetaStdDev = other.minThetaStdDev;
    this.maxThetaStdDev = other.maxThetaStdDev;
  }

  /**
   * Requires at least this many contributing tags. The count uses the {@link
   * FiducialTarget#fielded} flags. At least one fielded tag is always required. Use {@code
   * withMinTagCount(2)} to ignore single-tag MT1 estimates.
   */
  public PoseEstimateConfig withMinTagCount(int minTagCount) {
    this.minTagCount = Math.max(1, minTagCount);
    return this;
  }

  /**
   * Rejects single-tag estimates when the ambiguity of the fielded tag is above this value (0-1). A
   * value of 1 disables this check.
   */
  public PoseEstimateConfig withMaxSingleTagAmbiguity(double maxSingleTagAmbiguity) {
    this.maxSingleTagAmbiguity = Limelight.clampArg(maxSingleTagAmbiguity, 0, 1, 1);
    return this;
  }

  /**
   * Rejects single-tag estimates when the tag is farther than this distance in meters. Single-tag
   * estimates lose accuracy with distance faster than multi-tag estimates. Set this value tighter
   * than {@link #withMaxAvgTagDistance}. For example, allow multi-tag estimates to 6 m and
   * single-tag estimates to 2.5 m. Applies only when exactly one tag contributes. 0 disables this
   * check.
   */
  public PoseEstimateConfig withMaxSingleTagDistance(double maxSingleTagDistanceMeters) {
    this.maxSingleTagDistance = Limelight.clampArg(maxSingleTagDistanceMeters, 0, Double.MAX_VALUE);
    return this;
  }

  /**
   * Rejects estimates when the average tag distance is above this value in meters. A value of 0
   * disables this check.
   */
  public PoseEstimateConfig withMaxAvgTagDistance(double maxAvgTagDistanceMeters) {
    this.maxAvgTagDistance = Limelight.clampArg(maxAvgTagDistanceMeters, 0, Double.MAX_VALUE);
    return this;
  }

  /**
   * Rejects estimates when the average tag area (percentage of image) is below this value. A value
   * of 0 disables this check.
   */
  public PoseEstimateConfig withMinAvgTagArea(double minAvgTagArea) {
    this.minAvgTagArea = Limelight.clampArg(minAvgTagArea, 0, 100);
    return this;
  }

  /**
   * Rejects estimates outside the field. The bounds depend on the coordinate origin of the
   * estimate. Corner origins (wpiblue, wpired) span [0, length] x [0, width]. Centered origins span
   * +/-length/2 x +/-width/2.
   *
   * <p>A zero length or width disables this check.
   *
   * @param fieldLengthMeters Field length (x extent) in meters
   * @param fieldWidthMeters Field width (y extent) in meters
   */
  public PoseEstimateConfig withFieldBounds(double fieldLengthMeters, double fieldWidthMeters) {
    this.fieldLengthMeters = Limelight.clampArg(fieldLengthMeters, 0, Double.MAX_VALUE);
    this.fieldWidthMeters = Limelight.clampArg(fieldWidthMeters, 0, Double.MAX_VALUE);
    return this;
  }

  /**
   * Sets the margin for the field bounds check. The default is 0.5. A positive margin accepts poses
   * up to that distance outside the field. This allows small localization errors at the walls. A
   * negative margin requires poses at least that distance inside the field walls.
   *
   * @param marginMeters Margin in meters. Negative values are an inset. NaN becomes 0. Infinite
   *     values clamp to the largest finite margin
   */
  public PoseEstimateConfig withFieldBoundsMargin(double marginMeters) {
    this.fieldBoundsMarginMeters =
        Double.isNaN(marginMeters) ? 0 : clamp(marginMeters, -Double.MAX_VALUE, Double.MAX_VALUE);
    return this;
  }

  void applyTo(PoseEstimate estimate) {
    estimate.stdDevs = compute(estimate);
    estimate.rejectionFlags |= evaluate(estimate);
  }

  private int evaluate(PoseEstimate estimate) {
    int flags = 0;
    double maxFieldedAmbiguity = 0;
    for (FiducialTarget fiducial : estimate.rawFiducials) {
      if (fiducial.fielded) {
        maxFieldedAmbiguity = Math.max(maxFieldedAmbiguity, fiducial.ambiguity);
      }
    }
    int contributingTags = estimate.fieldedTagCount;

    if (contributingTags < minTagCount) {
      flags |= REJECT_TAG_COUNT;
    }
    if (maxSingleTagAmbiguity < 1.0
        && contributingTags == 1
        && maxFieldedAmbiguity > maxSingleTagAmbiguity) {
      flags |= REJECT_AMBIGUITY;
    }
    if (maxSingleTagDistance > 0
        && contributingTags == 1
        && estimate.avgTagDistanceMeters > maxSingleTagDistance) {
      flags |= REJECT_TAG_DISTANCE;
    }
    if (maxAvgTagDistance > 0 && estimate.avgTagDistanceMeters > maxAvgTagDistance) {
      flags |= REJECT_TAG_DISTANCE;
    }
    if (minAvgTagArea > 0 && estimate.avgTagAreaPercent < minAvgTagArea) {
      flags |= REJECT_TAG_AREA;
    }
    if (fieldLengthMeters > 0 && fieldWidthMeters > 0) {
      double x = estimate.pose.getX();
      double y = estimate.pose.getY();
      boolean outOfBounds;
      if (Limelight.centeredOrigin(estimate.type)) {
        outOfBounds =
            Math.abs(x) > fieldLengthMeters / 2 + fieldBoundsMarginMeters
                || Math.abs(y) > fieldWidthMeters / 2 + fieldBoundsMarginMeters;
      } else {
        outOfBounds =
            x < -fieldBoundsMarginMeters
                || x > fieldLengthMeters + fieldBoundsMarginMeters
                || y < -fieldBoundsMarginMeters
                || y > fieldWidthMeters + fieldBoundsMarginMeters;
      }
      if (outOfBounds) {
        flags |= REJECT_FIELD_BOUNDS;
      }
    }
    return flags;
  }

  /**
   * Returns the rejection flags as readable text, for example "TAG_COUNT|AMBIGUITY".
   *
   * @return The rejection flags as readable text, for example "TAG_COUNT|AMBIGUITY". Empty if the
   *     estimate was accepted
   */
  public static String describeRejection(int flags) {
    if (flags == 0) {
      return "";
    }
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < Limelight.REJECTION_FLAGS.length; i++) {
      appendFlag(sb, flags, Limelight.REJECTION_FLAGS[i], Limelight.REJECTION_NAMES[i]);
    }
    return sb.toString();
  }

  private static void appendFlag(StringBuilder sb, int flags, int flag, String name) {
    if ((flags & flag) != 0) {
      if (sb.length() > 0) {
        sb.append('|');
      }
      sb.append(name);
    }
  }

  // ---- Fusion standard deviations ----
  // xy = clamp(xyStdDev * distance^distExp / fieldedTagCount^tagExp, minXY, maxXY)

  /** A standard deviation so large that a pose estimator ignores the measurement. */
  public static final double UNTRUSTED = 9999999;

  private static final double MIN_XY_STD_DEV = 0.0001;

  /** Smallest heading standard deviation this configuration produces, in radians. */
  public static final double MIN_THETA_STD_DEV = 0.01;

  /**
   * The MegaTag1 defaults. The library uses these when you do not provide an MT1 configuration.
   * Single-tag estimates must pass the ambiguity gate (0.7) and the distance gate (3 m). MT1
   * computes the full pose from tag geometry only. Heading is not fused. XY standard deviation =
   * 0.5 * tagDistanceMeters / sqrt(fieldedTagCount), with a minimum of 0.0001 m. These are untuned
   * initial values. Tune them on your robot. Start from this configuration and chain changes:
   * {@code PoseEstimateConfig.defaultMT1().withFieldBounds(...)}.
   */
  public static PoseEstimateConfig defaultMT1() {
    return new PoseEstimateConfig()
        .withMaxSingleTagAmbiguity(0.7)
        .withMaxSingleTagDistance(3.0)
        .withStdDevXY(0.5);
  }

  /**
   * The MegaTag2 defaults. The library uses these when you do not provide an MT2 configuration.
   * There are no acceptance gates. Gyro-fused MT2 is robust, and structural validation still
   * applies. Heading is not fused. XY standard deviation = 0.3 * tagDistanceMeters /
   * sqrt(fieldedTagCount), with a minimum of 0.0001 m. These are untuned initial values. Tune them
   * on your robot. Start from this configuration and chain changes.
   */
  public static PoseEstimateConfig defaultMT2() {
    return new PoseEstimateConfig().withStdDevXY(0.3);
  }

  /**
   * Accepts every estimate that decoded with a pose and a timestamp. Uses a fixed 0.5 m XY standard
   * deviation. Heading is untrusted.
   */
  public static PoseEstimateConfig noFiltering() {
    return noFiltering(0.5, UNTRUSTED);
  }

  /**
   * Accepts every estimate that decoded with a pose and a timestamp. Uses the fixed standard
   * deviations that you select.
   *
   * @param xyStdDevMeters Fixed XY standard deviation in meters
   * @param thetaStdDevRadians Fixed heading standard deviation in radians. Pass {@link #UNTRUSTED}
   *     to exclude vision heading from fusion
   */
  public static PoseEstimateConfig noFiltering(double xyStdDevMeters, double thetaStdDevRadians) {
    return new PoseEstimateConfig()
        .withStdDevXY(xyStdDevMeters)
        .withStdDevTheta(thetaStdDevRadians)
        .withStdDevDistanceScaling(0)
        .withStdDevTagCountDivision(0);
  }

  private double xyStdDev = 0.3;
  private double thetaStdDev = UNTRUSTED;
  private double distanceExponent = 1.0;
  private double minScalingDistance = 0.0;
  private double maxScalingDistance = Double.MAX_VALUE;
  private double tagCountExponent = 0.5;
  private double minXYStdDev = MIN_XY_STD_DEV;
  private double maxXYStdDev = Double.MAX_VALUE;
  private double minThetaStdDev = MIN_THETA_STD_DEV;
  private double maxThetaStdDev = Double.MAX_VALUE;

  /**
   * Sets the base XY standard deviation in meters. When distance scaling is active, this value is
   * per meter of average tag distance.
   */
  public PoseEstimateConfig withStdDevXY(double baseMeters) {
    this.xyStdDev = Limelight.clampArg(baseMeters, MIN_XY_STD_DEV, UNTRUSTED, UNTRUSTED);
    return this;
  }

  /** Sets the base XY standard deviation. Clamps the computed value to [min, max] meters. */
  public PoseEstimateConfig withStdDevXY(double baseMeters, double minMeters, double maxMeters) {
    withStdDevXY(baseMeters);
    double lo = Limelight.clampArg(minMeters, MIN_XY_STD_DEV, UNTRUSTED, MIN_XY_STD_DEV);
    double hi = Limelight.clampArg(maxMeters, MIN_XY_STD_DEV, UNTRUSTED, UNTRUSTED);
    this.minXYStdDev = Math.min(lo, hi);
    this.maxXYStdDev = Math.max(lo, hi);
    return this;
  }

  /**
   * Sets the base heading standard deviation in radians. The default excludes vision heading from
   * fusion. Lower this value only to fuse vision heading. The minimum is {@link #MIN_THETA_STD_DEV}
   * radians.
   */
  public PoseEstimateConfig withStdDevTheta(double baseRadians) {
    this.thetaStdDev = Limelight.clampArg(baseRadians, MIN_THETA_STD_DEV, UNTRUSTED, UNTRUSTED);
    return this;
  }

  /**
   * Sets the base heading standard deviation. Clamps the computed value to [min, max] radians. The
   * minimum is {@link #MIN_THETA_STD_DEV} radians.
   */
  public PoseEstimateConfig withStdDevTheta(
      double baseRadians, double minRadians, double maxRadians) {
    withStdDevTheta(baseRadians);
    double lo = Limelight.clampArg(minRadians, MIN_THETA_STD_DEV, UNTRUSTED, MIN_THETA_STD_DEV);
    double hi = Limelight.clampArg(maxRadians, MIN_THETA_STD_DEV, UNTRUSTED, UNTRUSTED);
    this.minThetaStdDev = Math.min(lo, hi);
    this.maxThetaStdDev = Math.max(lo, hi);
    return this;
  }

  /**
   * Scales the standard deviations by avgTagDistance^exponent. 1 = linear (default). 2 = quadratic.
   * 0 = no distance scaling.
   */
  public PoseEstimateConfig withStdDevDistanceScaling(double exponent) {
    this.distanceExponent = Limelight.clampArg(exponent, 0, 10, 1);
    return this;
  }

  /**
   * Same as {@link #withStdDevDistanceScaling(double)}. Clamps the tag distance to [min, max]
   * meters before the exponent applies. Tags closer than min scale as if at min. Tags farther than
   * max scale as if at max.
   */
  public PoseEstimateConfig withStdDevDistanceScaling(
      double exponent, double minMeters, double maxMeters) {
    withStdDevDistanceScaling(exponent);
    double lo = Limelight.clampArg(minMeters, 0, Double.MAX_VALUE, 0);
    double hi = Limelight.clampArg(maxMeters, 0, Double.MAX_VALUE, Double.MAX_VALUE);
    this.minScalingDistance = Math.min(lo, hi);
    this.maxScalingDistance = Math.max(lo, hi);
    return this;
  }

  /**
   * Divides the standard deviations by fieldedTagCount^exponent. The default 0.5 divides by the
   * square root of the fielded tag count. 0 disables this.
   */
  public PoseEstimateConfig withStdDevTagCountDivision(double exponent) {
    this.tagCountExponent = Limelight.clampArg(exponent, 0, 10, 0.5);
    return this;
  }

  private Vector<N3> compute(PoseEstimate estimate) {
    double scale = 1.0;
    if (distanceExponent != 0) {
      if (!Double.isFinite(estimate.avgTagDistanceMeters) || estimate.avgTagDistanceMeters <= 0) {
        return VecBuilder.fill(UNTRUSTED, UNTRUSTED, UNTRUSTED);
      }
      double distance =
          clamp(estimate.avgTagDistanceMeters, minScalingDistance, maxScalingDistance);
      scale *= Math.pow(distance, distanceExponent);
    }
    if (tagCountExponent != 0 && estimate.fieldedTagCount > 1) {
      scale /= Math.pow(estimate.fieldedTagCount, tagCountExponent);
    }
    double xy = clamp(xyStdDev * scale, minXYStdDev, maxXYStdDev);
    double theta = clamp(thetaStdDev * scale, minThetaStdDev, maxThetaStdDev);
    return VecBuilder.fill(xy, xy, theta);
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
