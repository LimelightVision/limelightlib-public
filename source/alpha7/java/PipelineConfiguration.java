// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

import java.nio.charset.StandardCharsets;

/**
 * An immutable, validated pipeline configuration override. It holds the contents of a .vpr file.
 * Create it one time during robot initialization. All file IO and size validation happen at that
 * time. Then publish it at any time with {@link
 * Limelight#setPipelineConfigurationOverride(PipelineConfiguration)}:
 *
 * {@snippet :
 * // robotInit
 * PipelineConfiguration aiming = PipelineConfiguration.fromDeployFolder("aiming");
 * PipelineConfiguration intake = PipelineConfiguration.fromDeployFolder("intake");
 * // mid-match
 * camera.setPipelineConfigurationOverride(aiming);
 * }
 *
 * <p>A failed load produces an instance where {@link #isValid()} is false.
 */
public final class PipelineConfiguration {
  final String contents;
  private final LoadStatus status;

  // might add builder style api someday bh

  private PipelineConfiguration(String contents, LoadStatus status) {
    this.contents = contents;
    this.status = status;
  }

  /**
   * Wraps .vpr contents. Checks the {@link Limelight#MAX_PIPELINE_CONFIGURATION_OVERRIDE_BYTES}
   * size cap one time.
   *
   * @param contents The .vpr pipeline file contents
   * @return The wrapped configuration. Not valid when null, empty, or too large
   */
  public static PipelineConfiguration fromString(String contents) {
    if (contents == null || contents.isEmpty()) {
      Limelight.reportWarning("Limelight - pipeline configuration rejected: no contents");
      return new PipelineConfiguration(null, LoadStatus.NO_CONTENTS);
    }
    if (contents.getBytes(StandardCharsets.UTF_8).length
        > Limelight.MAX_PIPELINE_CONFIGURATION_OVERRIDE_BYTES) {
      Limelight.reportWarning(
          "Limelight - pipeline configuration rejected: larger than "
              + Limelight.MAX_PIPELINE_CONFIGURATION_OVERRIDE_BYTES
              + " bytes");
      return new PipelineConfiguration(null, LoadStatus.TOO_LARGE);
    }
    return new PipelineConfiguration(contents, LoadStatus.OK);
  }

  /**
   * Reads a .vpr pipeline file from the deploy directory of the robot program ({@code
   * src/main/deploy} in the robot project). If the read fails, this method prints a warning and
   * returns a configuration that is not valid.
   *
   * @param deployRelativePath The pipeline file relative to the deploy directory, for example
   *     "aiming" or "pipelines/aiming.vpr". ".vpr" is added when missing
   * @return The loaded configuration. Check {@link #isValid}
   */
  public static PipelineConfiguration fromDeployFolder(String deployRelativePath) {
    if (deployRelativePath == null || deployRelativePath.isEmpty()) {
      Limelight.reportWarning("Limelight - no pipeline configuration file name given");
      return new PipelineConfiguration(null, LoadStatus.NO_CONTENTS);
    }
    String contents =
        Limelight.readFileForNT(
            Limelight.deployPath(deployRelativePath, ".vpr", "pipeline configuration"),
            "pipeline configuration");
    if (contents == null) {
      return new PipelineConfiguration(null, LoadStatus.READ_FAILED);
    }
    return fromString(contents);
  }

  /**
   * Returns true if this holds a usable configuration.
   *
   * @return True if this holds a usable configuration
   */
  public boolean isValid() {
    return status == LoadStatus.OK;
  }

  /**
   * Returns why this configuration is or is not usable.
   *
   * @return Why this configuration is or is not usable
   */
  public LoadStatus getLoadStatus() {
    return status;
  }

  /**
   * Returns the configuration's size in bytes, or 0 when invalid.
   *
   * @return The configuration's size in bytes, or 0 when invalid
   */
  public int sizeBytes() {
    return contents == null ? 0 : contents.getBytes(StandardCharsets.UTF_8).length;
  }
}
