// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <cctype>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

#include "wpi/system/Errors.hpp"
#include "wpi/system/Filesystem.hpp"
namespace limelight {

namespace detail {

/** Size cap for a pipeline configuration override, in bytes. Mirrored as
 *  Limelight::MAX_PIPELINE_CONFIGURATION_OVERRIDE_BYTES. */
inline constexpr size_t kMaxPipelineConfigurationOverrideBytes = 4096;

/** Size cap for a shared field map, in bytes. Mirrored as
 *  Limelight::MAX_SHARED_MAP_BYTES. */
inline constexpr size_t kMaxSharedMapBytes = 262144;

// Loader failures print a single line to stdout (flushed, so it shows up in
// robot service logs) and never throw.
// Loader warnings go to the Driver Station console like other WPILib
// warnings.
inline void WarnLoader(const std::string& message) {
  WPILIB_ReportWarning("Limelight - {}", message);
  std::fflush(stdout);
}

inline bool EndsWithIgnoreCase(std::string_view text, std::string_view suffix) {
  if (suffix.size() > text.size()) {
    return false;
  }
  std::string_view tail = text.substr(text.size() - suffix.size());
  for (size_t i = 0; i < suffix.size(); i++) {
    if (std::tolower(static_cast<unsigned char>(tail[i])) !=
        std::tolower(static_cast<unsigned char>(suffix[i]))) {
      return false;
    }
  }
  return true;
}

inline std::optional<std::string> ReadFileForNT(
    const std::filesystem::path& path, const char* what);

inline std::optional<std::string> ReadDeployFileForNT(
    std::string_view deployRelativePath, std::string_view extension,
    const char* what) {
  try {
    std::string name{deployRelativePath};
    if (!EndsWithIgnoreCase(name, extension)) {
      name.append(extension);
    }
    std::filesystem::path relative{name};
    if (relative.is_absolute()) {
      WarnLoader(std::string{what} +
                 " path must be relative to the deploy directory: " + name);
      return std::nullopt;
    }
    return ReadFileForNT(
        std::filesystem::path{wpi::filesystem::GetDeployDirectory()} / relative,
        what);
  } catch (const std::exception& e) {
    WarnLoader(std::string{"could not resolve the deploy directory: "} +
               e.what());
    return std::nullopt;
  }
}

inline std::optional<std::string> ReadFileForNT(
    const std::filesystem::path& path, const char* what) {
  try {
    std::ifstream file{path, std::ios::binary};
    if (!file) {
      WarnLoader(std::string{"could not read "} + what + " file " +
                 path.string());
      return std::nullopt;
    }
    std::string contents{std::istreambuf_iterator<char>{file},
                         std::istreambuf_iterator<char>{}};
    if (file.bad()) {
      WarnLoader(std::string{"could not read "} + what + " file " +
                 path.string());
      return std::nullopt;
    }
    return contents;
  } catch (const std::exception& e) {
    WarnLoader(std::string{"could not read "} + what + " file: " + e.what());
    return std::nullopt;
  }
}

}  // namespace detail

}  // namespace limelight
