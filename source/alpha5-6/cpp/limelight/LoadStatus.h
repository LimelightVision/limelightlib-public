// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

namespace limelight {

/** Result of loading a Limelight::PipelineConfiguration or
 *  Limelight::FieldMap. */
enum class LoadStatus {
  /** Loaded and within the size cap */
  OK,
  /** No contents or no file name were given (empty) */
  NO_CONTENTS,
  /** The deploy-folder file could not be read */
  READ_FAILED,
  /** The contents exceed the size cap */
  TOO_LARGE
};

}  // namespace limelight
