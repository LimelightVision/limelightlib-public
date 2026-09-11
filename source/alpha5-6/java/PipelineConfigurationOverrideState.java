// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

/** Pipeline configuration override state reported by the camera. */
public enum PipelineConfigurationOverrideState {
  /** No override requested. */
  OFF,
  /** The override is applied. */
  ACTIVE,
  /** The camera web interface forced the override off. */
  FORCED_OFF,
  /** The override is enabled but no configuration is published. */
  NO_STRING,
  /** The published configuration failed to parse or was too large. */
  PARSE_ERROR,
  /** The camera reported a state this library does not know. */
  UNKNOWN
}
