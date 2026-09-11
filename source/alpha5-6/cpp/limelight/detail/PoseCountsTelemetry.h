// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "limelight/PoseEstimate.h"
#include "limelight/PoseEstimateConfig.h"
#include "wpi/nt/IntegerTopic.hpp"
#include "wpi/nt/NetworkTable.hpp"
namespace limelight {

namespace detail {

class PoseCountsTelemetry {
 public:
  explicit PoseCountsTelemetry(wpi::nt::NetworkTable& telemetryTable) {
    std::shared_ptr<wpi::nt::NetworkTable> countsTable =
        telemetryTable.GetSubTable("counts");
    m_processed = countsTable->GetIntegerTopic("processed").Publish();
    m_accepted = countsTable->GetIntegerTopic("accepted").Publish();
    m_rejected = countsTable->GetIntegerTopic("rejected").Publish();
    m_processed.Set(0);
    m_accepted.Set(0);
    m_rejected.Set(0);
    for (size_t i = 0; i < kRejectionFlags.size(); i++) {
      m_rejectionCounts[i] =
          countsTable->GetIntegerTopic(std::string{kRejectionNames[i]})
              .Publish();
      m_rejectionCounts[i].Set(0);
    }
  }

  void Count(const PoseEstimate& estimate) {
    m_processed.Set(++m_processedTotal);
    if (estimate.IsValid()) {
      m_accepted.Set(++m_acceptedTotal);
      return;
    }
    m_rejected.Set(++m_rejectedTotal);
    for (size_t i = 0; i < kRejectionFlags.size(); i++) {
      if ((estimate.rejectionFlags & kRejectionFlags[i]) != 0) {
        m_rejectionCounts[i].Set(++m_rejectionTotals[i]);
      }
    }
  }

 private:
  wpi::nt::IntegerPublisher m_processed;
  wpi::nt::IntegerPublisher m_accepted;
  wpi::nt::IntegerPublisher m_rejected;
  std::array<wpi::nt::IntegerPublisher, kRejectionFlags.size()>
      m_rejectionCounts;
  std::array<int64_t, kRejectionFlags.size()> m_rejectionTotals{};
  int64_t m_processedTotal = 0;
  int64_t m_acceptedTotal = 0;
  int64_t m_rejectedTotal = 0;
};

}  // namespace detail

}  // namespace limelight
