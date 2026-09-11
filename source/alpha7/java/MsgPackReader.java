// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package com.limelightvision;

import java.nio.charset.StandardCharsets;

/**
 * Minimal, allocation-light MessagePack reader covering the full msgpack spec (nil, bool, all
 * int/uint widths, float32/64, str, bin, array, map, with skippable ext types). Nil is read
 * leniently as 0 / "" / empty so absent optional data never throws.
 */
final class MsgPackReader {
  private static final int MAX_SKIP_DEPTH = 64;

  private final byte[] buf;
  private int pos;
  private int skipDepth;

  // The reader decodes in place; the caller's buffer is never modified or retained.
  @SuppressWarnings("PMD.ArrayIsStoredDirectly")
  MsgPackReader(byte[] buf) {
    this.buf = buf;
  }

  private int u8() {
    return buf[pos++] & 0xFF;
  }

  // Peeks at the next token without consuming it
  boolean nextIsMap() {
    int b = buf[pos] & 0xFF;
    return (b & 0xF0) == 0x80 || b == 0xDE || b == 0xDF;
  }

  private int u16() {
    int v = ((buf[pos] & 0xFF) << 8) | (buf[pos + 1] & 0xFF);
    pos += 2;
    return v;
  }

  private long u32() {
    long v =
        ((long) (buf[pos] & 0xFF) << 24)
            | ((buf[pos + 1] & 0xFF) << 16)
            | ((buf[pos + 2] & 0xFF) << 8)
            | (buf[pos + 3] & 0xFF);
    pos += 4;
    return v;
  }

  private long u64() {
    long v = 0;
    for (int i = 0; i < 8; i++) {
      v = (v << 8) | (buf[pos + i] & 0xFF);
    }
    pos += 8;
    return v;
  }

  // The envelope root must be a map. Anything else is a decode error.
  int expectMapHeader() {
    int b = u8();
    if ((b & 0xF0) == 0x80) {
      return b & 0x0F;
    }
    switch (b) {
      case 0xDE:
        return checkedCount(u16(), 2, "map");
      case 0xDF:
        return checkedCount((int) u32(), 2, "map");
      default:
        throw badToken("map", b);
    }
  }

  // The typed readers below are lenient about the value type. A known key
  // whose value has an unexpected type is skipped and reads as its default
  // (0, 0.0, "", or empty) so one surprising field never fails the frame.
  int readMapHeader() {
    int b = u8();
    if ((b & 0xF0) == 0x80) {
      return b & 0x0F;
    }
    switch (b) {
      case 0xC0:
        return 0; // nil -> empty map
      case 0xDE:
        return checkedCount(u16(), 2, "map");
      case 0xDF:
        return checkedCount((int) u32(), 2, "map");
      default:
        skipMismatched();
        return 0;
    }
  }

  // Rewinds to the token that did not match and skips the whole value
  private void skipMismatched() {
    pos--;
    skipValue();
  }

  int readArrayHeader() {
    int b = u8();
    if ((b & 0xF0) == 0x90) {
      return b & 0x0F;
    }
    switch (b) {
      case 0xC0:
        return 0; // nil -> empty array
      case 0xDC:
        return checkedCount(u16(), 1, "array");
      case 0xDD:
        return checkedCount((int) u32(), 1, "array");
      default:
        skipMismatched();
        return 0;
    }
  }

  String readString() {
    int b = u8();
    int len;
    if (b >= 0xA0 && b <= 0xBF) {
      len = b & 0x1F;
    } else {
      switch (b) {
        case 0xC0:
          return ""; // nil -> empty string
        case 0xD9:
          len = u8();
          break;
        case 0xDA:
          len = u16();
          break;
        case 0xDB:
          len = (int) u32();
          break;
        default:
          skipMismatched();
          return "";
      }
    }
    checkedCount(len, 1, "string");
    String s = new String(buf, pos, len, StandardCharsets.UTF_8);
    pos += len;
    return s;
  }

  // Rejects corrupt length prefixes before they drive a huge allocation or a
  // silent cursor desync (n items need >= n * minBytesPerItem remaining bytes)
  private int checkedCount(int count, int minBytesPerItem, String what) {
    if (count < 0 || (long) count * minBytesPerItem > buf.length - pos) {
      throw new IllegalStateException("corrupt " + what + " length " + count + " near byte " + pos);
    }
    return count;
  }

  long readLong() {
    int b = u8();
    if (b <= 0x7F) {
      return b; // positive fixint
    }
    if (b >= 0xE0) {
      return (byte) b; // negative fixint
    }
    switch (b) {
      case 0xC0:
        return 0; // nil
      case 0xC2:
        return 0; // false
      case 0xC3:
        return 1; // true
      case 0xCA:
        return (long) Float.intBitsToFloat((int) u32());
      case 0xCB:
        return (long) Double.longBitsToDouble(u64());
      case 0xCC:
        return u8();
      case 0xCD:
        return u16();
      case 0xCE:
        return u32();
      case 0xCF:
        return u64(); // uint64: values > Long.MAX_VALUE wrap, never produced in practice
      case 0xD0:
        return (byte) u8();
      case 0xD1:
        return (short) u16();
      case 0xD2:
        return (int) u32();
      case 0xD3:
        return u64();
      default:
        skipMismatched();
        return 0;
    }
  }

  double readDouble() {
    int b = u8();
    if (b <= 0x7F) {
      return b;
    }
    if (b >= 0xE0) {
      return (byte) b;
    }
    switch (b) {
      case 0xC0:
        return 0.0;
      case 0xC2:
        return 0.0;
      case 0xC3:
        return 1.0;
      case 0xCA:
        return Float.intBitsToFloat((int) u32());
      case 0xCB:
        return Double.longBitsToDouble(u64());
      case 0xCC:
        return u8();
      case 0xCD:
        return u16();
      case 0xCE:
        return u32();
      case 0xCF:
        return u64();
      case 0xD0:
        return (byte) u8();
      case 0xD1:
        return (short) u16();
      case 0xD2:
        return (int) u32();
      case 0xD3:
        return u64();
      default:
        skipMismatched();
        return 0.0;
    }
  }

  // Limelight.EMPTY is a shared zero-length array; nothing can be stored in it.
  @SuppressWarnings("PMD.MethodReturnsInternalArray")
  double[] readDoubleArray() {
    int n = readArrayHeader();
    if (n == 0) {
      return Limelight.EMPTY;
    }
    double[] out = new double[n];
    for (int i = 0; i < n; i++) {
      out[i] = readDouble();
    }
    return out;
  }

  /**
   * Reads a pose array. The camera always sends six values, all zero when there is no pose. A value
   * with fewer than six numbers, including a missing or malformed one, reads as six zeros, which is
   * the no-pose value. Longer arrays are kept as sent. Index access up to [5] is always safe.
   */
  double[] readPoseArray() {
    double[] out = readDoubleArray();
    return out.length < Limelight.POSE_ARRAY_LENGTH ? new double[Limelight.POSE_ARRAY_LENGTH] : out;
  }

  // Limelight.EMPTY_CORNERS is a shared zero-length array; nothing can be stored in it.
  @SuppressWarnings("PMD.MethodReturnsInternalArray")
  double[][] readPointArray() {
    int n = readArrayHeader();
    if (n == 0) {
      return Limelight.EMPTY_CORNERS;
    }
    double[][] out = new double[n][];
    for (int i = 0; i < n; i++) {
      out[i] = readDoubleArray();
    }
    return out;
  }

  void skipValue() {
    // A corrupt envelope of nested container headers would recurse once per
    // byte and StackOverflowError past decode()'s never-throws catch
    if (++skipDepth > MAX_SKIP_DEPTH) {
      throw new IllegalStateException("msgpack nested deeper than " + MAX_SKIP_DEPTH);
    }
    try {
      skipValueInner();
    } finally {
      skipDepth--;
    }
  }

  private void skipValueInner() {
    int b = u8();
    if (b <= 0x7F || b >= 0xE0) {
      return; // fixint
    }
    if (b >= 0xA0 && b <= 0xBF) {
      skipBytes(b & 0x1F); // fixstr
      return;
    }
    if (b >= 0x90 && b <= 0x9F) {
      skipValues(b & 0x0F); // fixarray
      return;
    }
    if (b >= 0x80 && b <= 0x8F) {
      skipValues(2 * (b & 0x0F)); // fixmap
      return;
    }
    switch (b) {
      case 0xC0:
      case 0xC2:
      case 0xC3:
        return;
      case 0xCC:
      case 0xD0:
        skipBytes(1);
        return;
      case 0xCD:
      case 0xD1:
        skipBytes(2);
        return;
      case 0xCE:
      case 0xD2:
      case 0xCA:
        skipBytes(4);
        return;
      case 0xCF:
      case 0xD3:
      case 0xCB:
        skipBytes(8);
        return;
      case 0xD9:
      case 0xC4:
        skipBytes(u8());
        return;
      case 0xDA:
      case 0xC5:
        skipBytes(u16());
        return;
      case 0xDB:
      case 0xC6:
        skipBytes((int) u32());
        return;
      case 0xD4:
        skipBytes(2);
        return; // fixext1
      case 0xD5:
        skipBytes(3);
        return; // fixext2
      case 0xD6:
        skipBytes(5);
        return; // fixext4
      case 0xD7:
        skipBytes(9);
        return; // fixext8
      case 0xD8:
        skipBytes(17);
        return; // fixext16
      case 0xC7:
        skipBytes(1 + u8());
        return;
      case 0xC8:
        skipBytes(1 + u16());
        return;
      case 0xC9:
        {
          long length = u32();
          if (length > Integer.MAX_VALUE - 1L) {
            throw new IllegalStateException("corrupt ext32 length " + length + " near byte " + pos);
          }
          skipBytes(1 + (int) length);
          return;
        }
      case 0xDC:
        skipValues(checkedCount(u16(), 1, "skipped array"));
        return;
      case 0xDD:
        skipValues(checkedCount((int) u32(), 1, "skipped array"));
        return;
      case 0xDE:
        skipValues(2 * checkedCount(u16(), 2, "skipped map"));
        return;
      case 0xDF:
        skipValues(2 * checkedCount((int) u32(), 2, "skipped map"));
        return;
      default:
        throw badToken("value", b);
    }
  }

  private void skipBytes(int count) {
    if (count < 0 || count > buf.length - pos) {
      throw new IllegalStateException("truncated msgpack value near byte " + pos);
    }
    pos += count;
  }

  private void skipValues(int count) {
    for (int i = 0; i < count; i++) {
      skipValue();
    }
  }

  private IllegalStateException badToken(String expected, int token) {
    return new IllegalStateException(
        "expected " + expected + ", got 0x" + Integer.toHexString(token) + " at byte " + (pos - 1));
  }
}
