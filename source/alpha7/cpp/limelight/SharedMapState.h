// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

namespace limelight {

/** Shared field map state reported by the camera. */
enum class SharedMapState {
  /** No shared map published */
  OFF,
  /** The shared map is in use */
  ACTIVE,
  /** The published map failed to parse or was too large */
  PARSE_ERROR,
  /** The camera reported a state this library does not know */
  UNKNOWN
};

}  // namespace limelight
