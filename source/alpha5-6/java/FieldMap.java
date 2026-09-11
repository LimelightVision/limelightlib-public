// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

import java.nio.charset.StandardCharsets;

/**
 * An immutable, validated shared field map. It holds the contents of an .fmap file. Create it one
 * time during robot initialization. All file IO and size validation happen at that time. Then
 * publish it at any time with {@link Limelight#setSharedMap(FieldMap)}:
 *
 * {@snippet :
 * // robotInit
 * FieldMap fieldMap = FieldMap.fromDeployFolder("field");
 * Limelight.setSharedMap(fieldMap);
 * }
 *
 * <p>A failed load produces an instance where {@link #isValid()} is false.
 */
public final class FieldMap {
  final String contents;
  private final LoadStatus status;

  private FieldMap(String contents, LoadStatus status) {
    this.contents = contents;
    this.status = status;
  }

  /**
   * Wraps .fmap contents. Checks the {@link Limelight#MAX_SHARED_MAP_BYTES} size cap one time.
   *
   * @param contents The .fmap field map file contents
   * @return The wrapped field map. Not valid when null, empty, or too large
   */
  public static FieldMap fromString(String contents) {
    if (contents == null || contents.isEmpty()) {
      Limelight.reportWarning("Limelight - field map rejected: no contents");
      return new FieldMap(null, LoadStatus.NO_CONTENTS);
    }
    if (contents.getBytes(StandardCharsets.UTF_8).length > Limelight.MAX_SHARED_MAP_BYTES) {
      Limelight.reportWarning(
          "Limelight - field map rejected: larger than "
              + Limelight.MAX_SHARED_MAP_BYTES
              + " bytes");
      return new FieldMap(null, LoadStatus.TOO_LARGE);
    }
    return new FieldMap(contents, LoadStatus.OK);
  }

  /**
   * Reads an .fmap field map file from the deploy directory of the robot program ({@code
   * src/main/deploy} in the robot project). If the read fails, this method prints a warning and
   * returns a field map that is not valid.
   *
   * @param deployRelativePath The field map file relative to the deploy directory, for example
   *     "field" or "maps/field.fmap". ".fmap" is added when missing
   * @return The loaded field map. Check {@link #isValid}
   */
  public static FieldMap fromDeployFolder(String deployRelativePath) {
    if (deployRelativePath == null || deployRelativePath.isEmpty()) {
      Limelight.reportWarning("Limelight - no field map file name given");
      return new FieldMap(null, LoadStatus.NO_CONTENTS);
    }
    String contents =
        Limelight.readFileForNT(
            Limelight.deployPath(deployRelativePath, ".fmap", "field map"), "field map");
    if (contents == null) {
      return new FieldMap(null, LoadStatus.READ_FAILED);
    }
    return fromString(contents);
  }

  /**
   * Returns true if this holds a usable field map.
   *
   * @return True if this holds a usable field map
   */
  public boolean isValid() {
    return status == LoadStatus.OK;
  }

  /**
   * Returns why this field map is or is not usable.
   *
   * @return Why this field map is or is not usable
   */
  public LoadStatus getLoadStatus() {
    return status;
  }

  /**
   * Returns the field map's size in bytes, or 0 when invalid.
   *
   * @return The field map's size in bytes, or 0 when invalid
   */
  public int sizeBytes() {
    return contents == null ? 0 : contents.getBytes(StandardCharsets.UTF_8).length;
  }
}
