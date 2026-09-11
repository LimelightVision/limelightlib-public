// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

namespace limelight {

/**
 * Health of a camera.
 */
enum class Status {
  /** A decodable frame has been received recently */
  OK,
  /** The camera has not sent a results envelope. Possible causes: the camera
   *  is off, the name is wrong, or MsgPack output is disabled */
  NO_DATA,
  /** The newest frame is older than the stale threshold. The camera is
   *  disconnected or has stopped */
  STALE,
  /** The newest frame did not decode. See LimelightResults::error */
  DECODE_ERROR
};

}  // namespace limelight
