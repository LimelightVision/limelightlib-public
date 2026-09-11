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
 * An immutable, validated shared field map. It holds the contents of an .fmap
 * file. Create it one time during robot initialization. All file IO and size
 * validation happen at that time. Then publish it at any time with
 * SetSharedMap(const FieldMap&):
 * @code
 * // RobotInit
 * using limelight::Limelight;
 * auto fieldMap = FieldMap::FromDeployFolder("field");
 * Limelight::SetSharedMap(fieldMap);
 * @endcode
 *
 * A failed load produces an instance where IsValid() is false.
 */
class FieldMap {
 public:
  /**
   * Wraps .fmap contents. Checks the Limelight::MAX_SHARED_MAP_BYTES size cap
   * one time.
   *
   * @param contents The .fmap field map file contents
   * @return The wrapped field map. Not valid when the contents are empty or
   *         too large
   */
  static FieldMap FromString(std::string_view contents) {
    if (contents.empty()) {
      detail::WarnLoader("field map rejected: no contents");
      return FieldMap{};
    }
    if (contents.size() > detail::kMaxSharedMapBytes) {
      detail::WarnLoader("field map rejected: larger than " +
                         std::to_string(detail::kMaxSharedMapBytes) + " bytes");
      FieldMap bad;
      bad.m_status = LoadStatus::TOO_LARGE;
      return bad;
    }
    FieldMap map;
    map.m_contents = std::string{contents};
    map.m_status = LoadStatus::OK;
    return map;
  }

  /**
   * Reads an .fmap field map file from the deploy directory of the robot
   * program (src/main/deploy in the robot project). If the read fails, this
   * method prints a warning and returns a field map that is not valid.
   *
   * @param deployRelativePath The field map file relative to the deploy
   *        directory, for example "field" or "maps/field.fmap". ".fmap" is
   *        added when missing
   * @return The loaded field map. Check IsValid()
   */
  static FieldMap FromDeployFolder(std::string_view deployRelativePath) {
    if (deployRelativePath.empty()) {
      detail::WarnLoader("no field map file name given");
      return FieldMap{};
    }
    std::optional<std::string> contents =
        detail::ReadDeployFileForNT(deployRelativePath, ".fmap", "field map");
    if (!contents) {
      FieldMap bad;
      bad.m_status = LoadStatus::READ_FAILED;
      return bad;
    }
    return FromString(*contents);
  }

  /** @return True if this holds a usable field map */
  bool IsValid() const { return m_status == LoadStatus::OK; }

  /** @return Why this field map is or is not usable */
  LoadStatus GetLoadStatus() const { return m_status; }

  /** @return The field map's size in bytes, or 0 when invalid */
  size_t SizeBytes() const { return m_contents.size(); }

 private:
  friend class Limelight;
  std::string m_contents;
  LoadStatus m_status = LoadStatus::NO_CONTENTS;
};

}  // namespace limelight
