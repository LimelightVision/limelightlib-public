// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "limelight/FiducialTarget.h"
#include "limelight/PoseEstimateConfig.h"
#include "limelight/PoseEstimateType.h"
#include "wpi/math/geometry/Pose2d.hpp"
#include "wpi/units/length.hpp"
#include "wpi/units/time.hpp"
#include "wpi/util/array.hpp"
namespace limelight {

/**
 * A robot pose estimate with the metadata needed for pose-estimator fusion.
 */
class PoseEstimate {
 public:
  wpi::math::Pose2d pose;
  /**
   * Latency-compensated capture timestamp in the local NetworkTables timebase.
   * Pass this value directly to AddVisionMeasurement in WPILib.
   */
  wpi::units::second_t timestamp{0.0};
  /** Total latency (capture + targeting) in milliseconds */
  wpi::units::millisecond_t latency{0.0};
  /** Camera-reported botpose tag count, telemetry only */
  int reportedTagCount = 0;
  /** Number of fielded tags. Used for validity, filtering, and standard
   *  deviation scaling */
  int fieldedTagCount = 0;
  /** Max distance between contributing tags in meters */
  wpi::units::meter_t tagSpan{0.0};
  /** Average distance to contributing tags in meters */
  wpi::units::meter_t avgTagDistance{0.0};
  /** Average area of contributing tags (percentage of image) */
  double avgTagAreaPercent = 0;
  /**
   * Fusion-ready standard deviations [x meters, y meters, theta radians]. The
   * PoseEstimateConfig of this camera computes them from tag distance and tag
   * count. Pass this value as the third argument to AddVisionMeasurement.
   */
  wpi::util::array<double, 3> stdDevs{PoseEstimateConfig::UNTRUSTED,
                                      PoseEstimateConfig::UNTRUSTED,
                                      PoseEstimateConfig::UNTRUSTED};
  /**
   * Standard deviations [x, y, z, roll, pitch, yaw] reported by the camera. The
   * camera averages them over a long window. Use them for telemetry only. Do
   * not give them to a pose estimator. Use stdDevs instead.
   */
  std::vector<double> reportedStdDevs;
  /** The fiducials visible in this frame */
  std::vector<FiducialTarget> rawFiducials;
  /** Which pose estimate output this is (origin + algorithm) */
  PoseEstimateType type = PoseEstimateType::MT1_WPIBLUE;
  /**
   * The frame index of the source camera. Estimates from the same camera with
   * the same index came from one frame. Do not fuse more than one pose type
   * from one frame. The index can reset when the camera restarts.
   */
  int64_t frameIndex = -1;
  /**
   * OR of the PoseEstimateConfig REJECT_* flags for every check this estimate
   * failed. 0 = accepted. Decode with PoseEstimateConfig::DescribeRejection.
   * The flags reflect the filter configured for the algorithm of this estimate.
   */
  int rejectionFlags = 0;

  /**
   * @return The visible fiducials that contributed to pose estimation. These
   *         have FiducialTarget::fielded set
   */
  std::vector<FiducialTarget> GetFieldedFiducials() const {
    std::vector<FiducialTarget> out;
    for (const FiducialTarget& fiducial : rawFiducials) {
      if (fiducial.fielded) {
        out.push_back(fiducial);
      }
    }
    return out;
  }

  /** @return True if this estimate was produced by the MegaTag2 algorithm */
  bool IsMT2() const { return detail::UsesMT2(type); }

  /** @return True if at least one fielded tag produced this estimate and no
   *          check rejected it. This does not show current camera health. It
   *          does not show whether the frame was already read. */
  bool IsValid() const { return fieldedTagCount > 0 && rejectionFlags == 0; }

  std::string ToString() const {
    std::string rejection;
    if (rejectionFlags != 0) {
      rejection =
          " REJECTED:" + PoseEstimateConfig::DescribeRejection(rejectionFlags);
    }
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "PoseEstimate(%.*s x=%.2f y=%.2f deg=%.1f fieldedTags=%d "
                  "reportedTags=%d avgDist=%.2f ts=%.3f%s)",
                  static_cast<int>(detail::PoseEstimateTypeName(type).size()),
                  detail::PoseEstimateTypeName(type).data(), pose.X().value(),
                  pose.Y().value(), pose.Rotation().Degrees().value(),
                  fieldedTagCount, reportedTagCount, avgTagDistance.value(),
                  timestamp.value(), rejection.c_str());
    return std::string{buf};
  }
};

// PoseEstimateConfig members that need the complete PoseEstimate type.

inline void PoseEstimateConfig::ApplyTo(PoseEstimate& estimate) const {
  estimate.stdDevs = Compute(estimate);
  estimate.rejectionFlags |= Evaluate(estimate);
}

inline int PoseEstimateConfig::Evaluate(const PoseEstimate& estimate) const {
  int flags = 0;
  double maxFieldedAmbiguity = 0;
  for (const FiducialTarget& fiducial : estimate.rawFiducials) {
    if (fiducial.fielded) {
      maxFieldedAmbiguity = std::max(maxFieldedAmbiguity, fiducial.ambiguity);
    }
  }
  int contributingTags = estimate.fieldedTagCount;

  if (contributingTags < m_minTagCount) {
    flags |= REJECT_TAG_COUNT;
  }
  if (m_maxSingleTagAmbiguity < 1.0 && contributingTags == 1) {
    if (maxFieldedAmbiguity > m_maxSingleTagAmbiguity) {
      flags |= REJECT_AMBIGUITY;
    }
  }
  if (m_maxSingleTagDistance > 0 && contributingTags == 1 &&
      estimate.avgTagDistance.value() > m_maxSingleTagDistance) {
    flags |= REJECT_TAG_DISTANCE;
  }
  if (m_maxAvgTagDistance > 0 &&
      estimate.avgTagDistance.value() > m_maxAvgTagDistance) {
    flags |= REJECT_TAG_DISTANCE;
  }
  if (m_minAvgTagArea > 0 && estimate.avgTagAreaPercent < m_minAvgTagArea) {
    flags |= REJECT_TAG_AREA;
  }
  if (m_fieldLengthMeters > 0 && m_fieldWidthMeters > 0) {
    double x = estimate.pose.X().value();
    double y = estimate.pose.Y().value();
    bool outOfBounds;
    if (detail::CenteredOrigin(estimate.type)) {
      outOfBounds =
          std::abs(x) > m_fieldLengthMeters / 2 + m_fieldBoundsMarginMeters ||
          std::abs(y) > m_fieldWidthMeters / 2 + m_fieldBoundsMarginMeters;
    } else {
      outOfBounds = x < -m_fieldBoundsMarginMeters ||
                    x > m_fieldLengthMeters + m_fieldBoundsMarginMeters ||
                    y < -m_fieldBoundsMarginMeters ||
                    y > m_fieldWidthMeters + m_fieldBoundsMarginMeters;
    }
    if (outOfBounds) {
      flags |= REJECT_FIELD_BOUNDS;
    }
  }
  return flags;
}

inline wpi::util::array<double, 3> PoseEstimateConfig::Compute(
    const PoseEstimate& estimate) const {
  double scale = 1.0;
  if (m_distanceExponent != 0) {
    if (!std::isfinite(estimate.avgTagDistance.value()) ||
        estimate.avgTagDistance.value() <= 0) {
      return {UNTRUSTED, UNTRUSTED, UNTRUSTED};
    }
    double distance = Clamp(estimate.avgTagDistance.value(),
                            m_minScalingDistance, m_maxScalingDistance);
    scale *= std::pow(distance, m_distanceExponent);
  }
  if (m_tagCountExponent != 0 && estimate.fieldedTagCount > 1) {
    scale /= std::pow(estimate.fieldedTagCount, m_tagCountExponent);
  }
  double xy = Clamp(m_xyStdDev * scale, m_minXYStdDev, m_maxXYStdDev);
  double theta =
      Clamp(m_thetaStdDev * scale, m_minThetaStdDev, m_maxThetaStdDev);
  return {xy, xy, theta};
}

}  // namespace limelight
