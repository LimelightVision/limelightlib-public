// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <array>
#include <cmath>
#include <span>
#include <string>

#include "limelight/PoseEstimate.h"
#include "limelight/PoseEstimateConfig.h"
#include "limelight/PoseEstimateType.h"
#include "wpi/math/geometry/Pose2d.hpp"
#include "wpi/math/geometry/struct/Pose2dStruct.hpp"
#include "wpi/nt/DoubleArrayTopic.hpp"
#include "wpi/nt/NetworkTable.hpp"
#include "wpi/nt/StringTopic.hpp"
#include "wpi/nt/StructArrayTopic.hpp"
namespace limelight {

namespace detail {

// Publishes Pose2d structs for AdvantageScope and Field2d arrays for
// Glass/Elastic.
class PoseTelemetry {
 public:
  PoseTelemetry(wpi::nt::NetworkTable& telemetryTable,
                wpi::nt::NetworkTable& fieldTable,
                const std::string& cameraName, PoseEstimateType type) {
    std::string typeName{PoseEstimateTypeName(type)};
    std::string prefix = typeName + "/";
    m_accepted =
        telemetryTable
            .GetStructArrayTopic<wpi::math::Pose2d>(prefix + "accepted")
            .Publish();
    m_rejected =
        telemetryTable
            .GetStructArrayTopic<wpi::math::Pose2d>(prefix + "rejected")
            .Publish();
    m_rejectionReasons =
        telemetryTable.GetStringTopic(prefix + "rejectionReasons").Publish();
    m_fieldAccepted =
        fieldTable.GetDoubleArrayTopic(cameraName + "-" + typeName).Publish();
    m_fieldRejected =
        fieldTable
            .GetDoubleArrayTopic(cameraName + "-" + typeName + "-rejected")
            .Publish();
  }

  void Publish(const PoseEstimate& estimate) {
    bool drawRejected =
        estimate.fieldedTagCount > 0 && estimate.rejectionFlags != 0 &&
        (estimate.rejectionFlags & PoseEstimateConfig::REJECT_MISSING_POSE) ==
            0 &&
        std::isfinite(estimate.pose.X().value()) &&
        std::isfinite(estimate.pose.Y().value()) &&
        std::isfinite(estimate.pose.Rotation().Radians().value());
    std::span<const wpi::math::Pose2d> onePose{&estimate.pose, 1};
    std::span<const wpi::math::Pose2d> noPoses{};
    m_accepted.Set(estimate.IsValid() ? onePose : noPoses);
    m_rejected.Set(drawRejected ? onePose : noPoses);
    m_rejectionReasons.Set(
        PoseEstimateConfig::DescribeRejection(estimate.rejectionFlags));
    std::array<double, 3> fieldPose = FieldPose(estimate.pose);
    std::span<const double> noDoubles{};
    m_fieldAccepted.Set(estimate.IsValid() ? std::span<const double>{fieldPose}
                                           : noDoubles);
    m_fieldRejected.Set(drawRejected ? std::span<const double>{fieldPose}
                                     : noDoubles);
  }

  void Clear() {
    m_accepted.Set({});
    m_rejected.Set(std::span<const wpi::math::Pose2d>{});
    m_rejectionReasons.Set("");
    m_fieldAccepted.Set({});
    m_fieldRejected.Set(std::span<const double>{});
  }

 private:
  static std::array<double, 3> FieldPose(const wpi::math::Pose2d& pose) {
    return {pose.X().value(), pose.Y().value(),
            pose.Rotation().Degrees().value()};
  }

  wpi::nt::StructArrayPublisher<wpi::math::Pose2d> m_accepted;
  wpi::nt::StructArrayPublisher<wpi::math::Pose2d> m_rejected;
  wpi::nt::StringPublisher m_rejectionReasons;
  wpi::nt::DoubleArrayPublisher m_fieldAccepted;
  wpi::nt::DoubleArrayPublisher m_fieldRejected;
};

}  // namespace detail

}  // namespace limelight
