// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

/** Result of loading a {@link PipelineConfiguration} or {@link FieldMap}. */
public enum LoadStatus {
  /** Loaded and within the size cap. */
  OK,
  /** No contents or no file name were given (null or empty). */
  NO_CONTENTS,
  /** The deploy-folder file could not be read. */
  READ_FAILED,
  /** The contents exceed the size cap. */
  TOO_LARGE
}
