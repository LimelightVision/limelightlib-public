// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>

#include "limelight/LoadStatus.h"
#include "limelight/detail/Loaders.h"
namespace limelight {

/**
 * An immutable, validated pipeline configuration override. It holds the
 * contents of a .vpr file. Create it one time during robot initialization.
 * All file IO and size validation happen at that time. Then publish it at
 * any time with SetPipelineConfigurationOverride(const
 * PipelineConfiguration&).
 * @code
 * // RobotInit
 * using limelight::Limelight;
 * auto aiming = PipelineConfiguration::FromDeployFolder("aiming");
 * auto intake = PipelineConfiguration::FromDeployFolder("intake");
 * // mid-match
 * camera.SetPipelineConfigurationOverride(aiming);
 * @endcode
 *
 * A failed load produces an instance where IsValid() is false.
 */
class PipelineConfiguration {
 public:
  /**
   * Wraps .vpr contents. Checks the
   * Limelight::MAX_PIPELINE_CONFIGURATION_OVERRIDE_BYTES size cap one time.
   *
   * @param contents The .vpr pipeline file contents
   * @return The wrapped configuration. Not valid when the contents are empty
   *         or too large
   */
  static PipelineConfiguration FromString(std::string_view contents) {
    if (contents.empty()) {
      detail::WarnLoader("pipeline configuration rejected: no contents");
      return PipelineConfiguration{};
    }
    if (contents.size() > detail::kMaxPipelineConfigurationOverrideBytes) {
      detail::WarnLoader(
          "pipeline configuration rejected: larger than " +
          std::to_string(detail::kMaxPipelineConfigurationOverrideBytes) +
          " bytes");
      PipelineConfiguration bad;
      bad.m_status = LoadStatus::TOO_LARGE;
      return bad;
    }
    PipelineConfiguration config;
    config.m_contents = std::string{contents};
    config.m_status = LoadStatus::OK;
    return config;
  }

  /**
   * Reads a .vpr pipeline file from the deploy directory of the robot program
   * (src/main/deploy in the robot project). If the read fails, this method
   * prints a warning and returns a configuration that is not valid.
   *
   * @param deployRelativePath The pipeline file relative to the deploy
   *        directory, for example "aiming" or "pipelines/aiming.vpr". ".vpr"
   * is added when missing
   * @return The loaded configuration. Check IsValid()
   */
  static PipelineConfiguration FromDeployFolder(
      std::string_view deployRelativePath) {
    if (deployRelativePath.empty()) {
      detail::WarnLoader("no pipeline configuration file name given");
      return PipelineConfiguration{};
    }
    std::optional<std::string> contents = detail::ReadDeployFileForNT(
        deployRelativePath, ".vpr", "pipeline configuration");
    if (!contents) {
      PipelineConfiguration bad;
      bad.m_status = LoadStatus::READ_FAILED;
      return bad;
    }
    return FromString(*contents);
  }

  /** @return True if this holds a usable configuration */
  bool IsValid() const { return m_status == LoadStatus::OK; }

  /** @return Why this configuration is or is not usable */
  LoadStatus GetLoadStatus() const { return m_status; }

  /** @return The configuration's size in bytes, or 0 when invalid */
  size_t SizeBytes() const { return m_contents.size(); }

 private:
  friend class Limelight;
  std::string m_contents;
  LoadStatus m_status = LoadStatus::NO_CONTENTS;
};

}  // namespace limelight
