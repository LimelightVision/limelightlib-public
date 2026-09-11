// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <string_view>

#include "limelight/detail/Math.h"
#include "wpi/units/angle.hpp"
#include "wpi/units/length.hpp"
#include "wpi/util/array.hpp"
namespace limelight {

class Limelight;

class PoseEstimate;

/**
 * A reusable configuration object for rejection filters and standard
 * deviation scaling. Start from a factory (DefaultMT1(), DefaultMT2(),
 * NoFiltering()) or from a new instance. Chain the With* methods. Attach the
 * result with Limelight::WithPoseEstimateConfig_MT1() or
 * Limelight::WithPoseEstimateConfig_MT2().
 *
 * A new instance accepts every estimate that decoded with a pose and a
 * timestamp and has at least one fielded tag. It uses the same standard
 * deviation model as DefaultMT2().
 *
 * @code
 * using namespace wpi::units::literals;
 * using limelight::Limelight;
 * using limelight::PoseEstimateConfig;
 * Limelight camera{"limelight"};
 * camera
 *     .WithPoseEstimateConfig_MT1(PoseEstimateConfig::DefaultMT1()
 *         .WithMinTagCount(2)
 *         .WithFieldBounds(17.55_m, 8.05_m))
 *     .WithPoseEstimateConfig_MT2(PoseEstimateConfig::DefaultMT2()
 *         .WithMaxAvgTagDistance(4_m)
 *         .WithFieldBounds(17.55_m, 8.05_m));
 * @endcode
 */
class PoseEstimateConfig {
 public:
  /** Rejection flag: too few contributing tags */
  static constexpr int REJECT_TAG_COUNT = 1 << 0;
  /** Rejection flag: single-tag ambiguity above the configured maximum */
  static constexpr int REJECT_AMBIGUITY = 1 << 1;
  /** Rejection flag: tag distance above a configured maximum */
  static constexpr int REJECT_TAG_DISTANCE = 1 << 2;
  /** Rejection flag: average tag area below the configured minimum */
  static constexpr int REJECT_TAG_AREA = 1 << 3;
  /** Rejection flag: pose outside the configured field bounds */
  static constexpr int REJECT_FIELD_BOUNDS = 1 << 4;

  /** Rejection flag set by the library: the source envelope had a decode
   *  error */
  static constexpr int REJECT_DECODE_ERROR = 1 << 5;

  /** Rejection flag set by the library: a value needed for fusion was NaN or
   *  infinite */
  static constexpr int REJECT_NONFINITE = 1 << 6;

  /** Rejection flag set by the library: no usable capture timestamp. For
   *  example, an envelope decoded with Limelight::Decode(bytes) has no receive
   *  time */
  static constexpr int REJECT_NO_TIMESTAMP = 1 << 7;

  /** Rejection flag set by the library: the pose array was absent or too
   *  short, it was the all-zero value the camera sends when it has no
   *  estimate, or the estimate type is not known */
  static constexpr int REJECT_MISSING_POSE = 1 << 8;

  /** Rejection flag set by the library: tags contributed but the reported tag
   *  distance was zero or negative */
  static constexpr int REJECT_BAD_METADATA = 1 << 9;

  /** Rejection flag set by the library: fiducials were visible but none
   *  contributed to pose estimation */
  static constexpr int REJECT_NO_FIELDED_TAGS = 1 << 11;

  /** A standard deviation so large that a pose estimator ignores the
   *  measurement */
  static constexpr double UNTRUSTED = 9999999;
  /** Smallest heading standard deviation this configuration produces, in
   *  radians */
  static constexpr double MIN_THETA_STD_DEV = 0.01;

  /**
   * Requires at least this many contributing tags. The count uses the
   * FiducialTarget::fielded flags. At least one fielded tag is always required.
   * Use WithMinTagCount(2) to ignore single-tag MT1 estimates.
   */
  PoseEstimateConfig& WithMinTagCount(int minTagCount) {
    m_minTagCount = std::max(1, minTagCount);
    return *this;
  }

  /**
   * Rejects single-tag estimates when the ambiguity of the fielded tag is above
   * this value (0-1). A value of 1 disables this check.
   */
  PoseEstimateConfig& WithMaxSingleTagAmbiguity(double maxSingleTagAmbiguity) {
    m_maxSingleTagAmbiguity = detail::ClampArg(maxSingleTagAmbiguity, 0, 1, 1);
    return *this;
  }

  /**
   * Rejects single-tag estimates when the tag is farther than this distance in
   * meters. Single-tag estimates lose accuracy with distance faster than
   * multi-tag estimates. Set this value tighter than WithMaxAvgTagDistance().
   * For example, allow multi-tag estimates to 6 m and single-tag estimates to
   * 2.5 m. Applies only when exactly one tag contributes. 0 disables this
   * check.
   */
  PoseEstimateConfig& WithMaxSingleTagDistance(
      wpi::units::meter_t maxSingleTagDistance) {
    m_maxSingleTagDistance = detail::ClampArg(
        maxSingleTagDistance.value(), 0, std::numeric_limits<double>::max());
    return *this;
  }

  /** Rejects estimates when the average tag distance is above this value in
   *  meters. A value of 0 disables this check. */
  PoseEstimateConfig& WithMaxAvgTagDistance(
      wpi::units::meter_t maxAvgTagDistance) {
    m_maxAvgTagDistance = detail::ClampArg(maxAvgTagDistance.value(), 0,
                                           std::numeric_limits<double>::max());
    return *this;
  }

  /** Rejects estimates when the average tag area (percentage of image) is
   *  below this value. A value of 0 disables this check. */
  PoseEstimateConfig& WithMinAvgTagArea(double minAvgTagArea) {
    m_minAvgTagArea = detail::ClampArg(minAvgTagArea, 0, 100);
    return *this;
  }

  /**
   * Rejects estimates outside the field. The bounds depend on the coordinate
   * origin of the estimate. Corner origins (wpiblue, wpired) span [0, length] x
   * [0, width]. Centered origins span +/-length/2 x +/-width/2.
   *
   * A zero length or width disables this check.
   *
   * @param fieldLength Field length (x extent) in meters
   * @param fieldWidth Field width (y extent) in meters
   */
  PoseEstimateConfig& WithFieldBounds(wpi::units::meter_t fieldLength,
                                      wpi::units::meter_t fieldWidth) {
    m_fieldLengthMeters = detail::ClampArg(fieldLength.value(), 0,
                                           std::numeric_limits<double>::max());
    m_fieldWidthMeters = detail::ClampArg(fieldWidth.value(), 0,
                                          std::numeric_limits<double>::max());
    return *this;
  }

  /**
   * Sets the margin for the field bounds check. The default is 0.5. A positive
   * margin accepts poses up to that distance outside the field. This allows
   * small localization errors at the walls. A negative margin requires poses
   * at least that distance inside the field walls.
   *
   * @param margin Margin in meters. Negative values are an inset. NaN
   *        becomes 0. Infinite values clamp to the largest finite margin
   */
  PoseEstimateConfig& WithFieldBoundsMargin(wpi::units::meter_t margin) {
    m_fieldBoundsMarginMeters =
        std::isnan(margin.value())
            ? 0
            : std::clamp(margin.value(), -std::numeric_limits<double>::max(),
                         std::numeric_limits<double>::max());
    return *this;
  }

  /**
   * The MegaTag1 defaults. The library uses these when you do not provide an
   * MT1 configuration. Single-tag estimates must pass the ambiguity gate (0.7)
   * and the distance gate (3 m). MT1 computes the full pose from tag geometry
   * only. Heading is not fused. XY standard deviation = 0.5 * tagDistanceMeters
   * / sqrt(fieldedTagCount), with a minimum of 0.0001 m. These are untuned
   * initial values. Tune them on your robot. Start from this configuration and
   * chain changes: PoseEstimateConfig::DefaultMT1().WithFieldBounds(...).
   */
  static PoseEstimateConfig DefaultMT1() {
    PoseEstimateConfig config;
    config.WithMaxSingleTagAmbiguity(0.7)
        .WithMaxSingleTagDistance(wpi::units::meter_t{3.0})
        .WithStdDevXY(wpi::units::meter_t{0.5});
    return config;
  }

  /**
   * The MegaTag2 defaults. The library uses these when you do not provide an
   * MT2 configuration. There are no acceptance gates. Gyro-fused MT2 is robust,
   * and structural validation still applies. Heading is not fused. XY standard
   * deviation = 0.3 * tagDistanceMeters / sqrt(fieldedTagCount), with a minimum
   * of 0.0001 m. These are untuned initial values. Tune them on your robot.
   * Start from this configuration and chain changes.
   */
  static PoseEstimateConfig DefaultMT2() {
    PoseEstimateConfig config;
    config.WithStdDevXY(wpi::units::meter_t{0.3});
    return config;
  }

  /**
   * Accepts every estimate that decoded with a pose and a timestamp. Uses a
   * fixed 0.5 m XY standard deviation. Heading is untrusted.
   */
  static PoseEstimateConfig NoFiltering() {
    return NoFiltering(wpi::units::meter_t{0.5},
                       wpi::units::radian_t{UNTRUSTED});
  }

  /**
   * Accepts every estimate that decoded with a pose and a timestamp. Uses the
   * fixed standard deviations that you select.
   *
   * @param xyStdDev Fixed XY standard deviation in meters
   * @param thetaStdDev Fixed heading standard deviation in radians. Pass
   *        UNTRUSTED to exclude vision heading from fusion
   */
  static PoseEstimateConfig NoFiltering(wpi::units::meter_t xyStdDev,
                                        wpi::units::radian_t thetaStdDev) {
    PoseEstimateConfig config;
    config.WithStdDevXY(xyStdDev)
        .WithStdDevTheta(thetaStdDev)
        .WithStdDevDistanceScaling(0)
        .WithStdDevTagCountDivision(0);
    return config;
  }

  /** Sets the base XY standard deviation in meters. When distance scaling is
   *  active, this value is per meter of average tag distance. */
  PoseEstimateConfig& WithStdDevXY(wpi::units::meter_t base) {
    m_xyStdDev =
        detail::ClampArg(base.value(), kMinXYStdDev, UNTRUSTED, UNTRUSTED);
    return *this;
  }

  /** Sets the base XY standard deviation. Clamps the computed value to
   *  [min, max] meters. */
  PoseEstimateConfig& WithStdDevXY(wpi::units::meter_t base,
                                   wpi::units::meter_t min,
                                   wpi::units::meter_t max) {
    WithStdDevXY(base);
    double lo =
        detail::ClampArg(min.value(), kMinXYStdDev, UNTRUSTED, kMinXYStdDev);
    double hi =
        detail::ClampArg(max.value(), kMinXYStdDev, UNTRUSTED, UNTRUSTED);
    m_minXYStdDev = std::min(lo, hi);
    m_maxXYStdDev = std::max(lo, hi);
    return *this;
  }

  /** Sets the base heading standard deviation in radians. The default excludes
   *  vision heading from fusion. Lower this value only to fuse vision heading.
   *  The minimum is MIN_THETA_STD_DEV radians. */
  PoseEstimateConfig& WithStdDevTheta(wpi::units::radian_t base) {
    m_thetaStdDev =
        detail::ClampArg(base.value(), MIN_THETA_STD_DEV, UNTRUSTED, UNTRUSTED);
    return *this;
  }

  /** Sets the base heading standard deviation. Clamps the computed value to
   *  [min, max] radians. The minimum is MIN_THETA_STD_DEV radians. */
  PoseEstimateConfig& WithStdDevTheta(wpi::units::radian_t base,
                                      wpi::units::radian_t min,
                                      wpi::units::radian_t max) {
    WithStdDevTheta(base);
    double lo = detail::ClampArg(min.value(), MIN_THETA_STD_DEV, UNTRUSTED,
                                 MIN_THETA_STD_DEV);
    double hi =
        detail::ClampArg(max.value(), MIN_THETA_STD_DEV, UNTRUSTED, UNTRUSTED);
    m_minThetaStdDev = std::min(lo, hi);
    m_maxThetaStdDev = std::max(lo, hi);
    return *this;
  }

  /**
   * Scales the standard deviations by avgTagDistance^exponent. 1 = linear
   * (default). 2 = quadratic. 0 = no distance scaling.
   */
  PoseEstimateConfig& WithStdDevDistanceScaling(double exponent) {
    m_distanceExponent = detail::ClampArg(exponent, 0, 10, 1);
    return *this;
  }

  /** Same as WithStdDevDistanceScaling(double). Clamps the tag distance to
   *  [min, max] meters before the exponent applies. Tags closer than min scale
   *  as if at min. Tags farther than max scale as if at max. */
  PoseEstimateConfig& WithStdDevDistanceScaling(double exponent,
                                                wpi::units::meter_t min,
                                                wpi::units::meter_t max) {
    WithStdDevDistanceScaling(exponent);
    double lo =
        detail::ClampArg(min.value(), 0, std::numeric_limits<double>::max(), 0);
    double hi =
        detail::ClampArg(max.value(), 0, std::numeric_limits<double>::max(),
                         std::numeric_limits<double>::max());
    m_minScalingDistance = std::min(lo, hi);
    m_maxScalingDistance = std::max(lo, hi);
    return *this;
  }

  /** Divides the standard deviations by fieldedTagCount^exponent. The default
   *  0.5 divides by the square root of the fielded tag count. 0 disables this.
   */
  PoseEstimateConfig& WithStdDevTagCountDivision(double exponent) {
    m_tagCountExponent = detail::ClampArg(exponent, 0, 10, 0.5);
    return *this;
  }

  /**
   * @return The rejection flags as readable text, for example
   *         "TAG_COUNT|AMBIGUITY". Empty if the estimate was accepted
   */
  static std::string DescribeRejection(int flags);

 private:
  friend class limelight::Limelight;

  void ApplyTo(PoseEstimate& estimate) const;
  int Evaluate(const PoseEstimate& estimate) const;
  wpi::util::array<double, 3> Compute(const PoseEstimate& estimate) const;

  static constexpr double Clamp(double value, double min, double max) {
    return std::max(min, std::min(max, value));
  }

  static constexpr double kMinXYStdDev = 0.0001;

  int m_minTagCount = 1;
  double m_maxSingleTagAmbiguity = 1.0;
  double m_maxSingleTagDistance = 0;
  double m_maxAvgTagDistance = 0;
  double m_minAvgTagArea = 0;
  double m_fieldLengthMeters = 0;
  double m_fieldWidthMeters = 0;
  double m_fieldBoundsMarginMeters = 0.5;

  // xy = clamp(xyStdDev * distance^distExp / fieldedTagCount^tagExp,
  //            minXY, maxXY)
  double m_xyStdDev = 0.3;
  double m_thetaStdDev = UNTRUSTED;
  double m_distanceExponent = 1.0;
  double m_minScalingDistance = 0.0;
  double m_maxScalingDistance = std::numeric_limits<double>::max();
  double m_tagCountExponent = 0.5;
  double m_minXYStdDev = kMinXYStdDev;
  double m_maxXYStdDev = std::numeric_limits<double>::max();
  double m_minThetaStdDev = MIN_THETA_STD_DEV;
  double m_maxThetaStdDev = std::numeric_limits<double>::max();
};

namespace detail {

inline constexpr std::array<int, 11> kRejectionFlags = {
    PoseEstimateConfig::REJECT_TAG_COUNT,
    PoseEstimateConfig::REJECT_AMBIGUITY,
    PoseEstimateConfig::REJECT_TAG_DISTANCE,
    PoseEstimateConfig::REJECT_TAG_AREA,
    PoseEstimateConfig::REJECT_FIELD_BOUNDS,
    PoseEstimateConfig::REJECT_DECODE_ERROR,
    PoseEstimateConfig::REJECT_NONFINITE,
    PoseEstimateConfig::REJECT_NO_TIMESTAMP,
    PoseEstimateConfig::REJECT_MISSING_POSE,
    PoseEstimateConfig::REJECT_BAD_METADATA,
    PoseEstimateConfig::REJECT_NO_FIELDED_TAGS};

inline constexpr std::array<std::string_view, 11> kRejectionNames = {
    "TAG_COUNT",    "AMBIGUITY",    "TAG_DISTANCE",   "TAG_AREA",
    "FIELD_BOUNDS", "DECODE_ERROR", "NONFINITE",      "NO_TIMESTAMP",
    "MISSING_POSE", "BAD_METADATA", "NO_FIELDED_TAGS"};

}  // namespace detail

inline std::string PoseEstimateConfig::DescribeRejection(int flags) {
  if (flags == 0) {
    return "";
  }
  std::string out;
  for (size_t i = 0; i < detail::kRejectionFlags.size(); i++) {
    if ((flags & detail::kRejectionFlags[i]) != 0) {
      if (!out.empty()) {
        out.push_back('|');
      }
      out.append(detail::kRejectionNames[i]);
    }
  }
  return out;
}

}  // namespace limelight
