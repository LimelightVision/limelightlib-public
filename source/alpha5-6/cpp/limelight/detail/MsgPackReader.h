// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "limelight/Geometry.h"

namespace limelight {

namespace detail {

// Minimal, allocation-light MessagePack reader covering the full msgpack spec
// (nil, bool, all int/uint widths, float32/64,
// str, bin, array, map, with skippable ext types). Nil is read leniently as
// 0 / "" / empty so absent optional data never throws.
class MsgPackReader {
 public:
  explicit MsgPackReader(std::span<const uint8_t> buf) : m_buf{buf} {}

  // Peeks at the next token without consuming it
  bool NextIsMap() const {
    RequireBytes(1);
    int b = m_buf[m_pos];
    return (b & 0xF0) == 0x80 || b == 0xDE || b == 0xDF;
  }

  // The envelope root must be a map. Anything else is a decode error.
  int ExpectMapHeader() {
    int b = U8();
    if ((b & 0xF0) == 0x80) {
      return b & 0x0F;
    }
    switch (b) {
      case 0xDE:
        return CheckedCount(U16(), 2, "map");
      case 0xDF:
        return CheckedCount(static_cast<int>(U32()), 2, "map");
      default:
        throw BadToken("map", b);
    }
  }

  // The typed readers below are lenient about the value type. A known key
  // whose value has an unexpected type is skipped and reads as its default
  // (0, 0.0, "", or empty) so one surprising field never fails the frame.
  int ReadMapHeader() {
    int b = U8();
    if ((b & 0xF0) == 0x80) {
      return b & 0x0F;
    }
    switch (b) {
      case 0xC0:
        return 0;  // nil -> empty map
      case 0xDE:
        return CheckedCount(U16(), 2, "map");
      case 0xDF:
        return CheckedCount(static_cast<int>(U32()), 2, "map");
      default:
        SkipMismatched();
        return 0;
    }
  }

  // Rewinds to the token that did not match and skips the whole value
  void SkipMismatched() {
    m_pos--;
    SkipValue();
  }

  int ReadArrayHeader() {
    int b = U8();
    if ((b & 0xF0) == 0x90) {
      return b & 0x0F;
    }
    switch (b) {
      case 0xC0:
        return 0;  // nil -> empty array
      case 0xDC:
        return CheckedCount(U16(), 1, "array");
      case 0xDD:
        return CheckedCount(static_cast<int>(U32()), 1, "array");
      default:
        SkipMismatched();
        return 0;
    }
  }

  std::string_view ReadString() {
    int b = U8();
    int len;
    if (b >= 0xA0 && b <= 0xBF) {
      len = b & 0x1F;
    } else {
      switch (b) {
        case 0xC0:
          return {};  // nil -> empty string
        case 0xD9:
          len = U8();
          break;
        case 0xDA:
          len = U16();
          break;
        case 0xDB:
          len = static_cast<int>(U32());
          break;
        default:
          SkipMismatched();
          return {};
      }
    }
    CheckedCount(len, 1, "string");
    std::string_view s{reinterpret_cast<const char*>(m_buf.data()) + m_pos,
                       static_cast<size_t>(len)};
    m_pos += static_cast<size_t>(len);
    return s;
  }

  int64_t ReadLong() {
    int b = U8();
    if (b <= 0x7F) {
      return b;  // positive fixint
    }
    if (b >= 0xE0) {
      return static_cast<int8_t>(b);  // negative fixint
    }
    switch (b) {
      case 0xC0:
        return 0;  // nil
      case 0xC2:
        return 0;  // false
      case 0xC3:
        return 1;  // true
      case 0xCA:
        return FloatToInt64(Float32());
      case 0xCB:
        return FloatToInt64(Float64());
      case 0xCC:
        return U8();
      case 0xCD:
        return U16();
      case 0xCE:
        return static_cast<int64_t>(U32());
      case 0xCF:
        // uint64: values > INT64_MAX wrap, never produced in practice
        return static_cast<int64_t>(U64());
      case 0xD0:
        return static_cast<int8_t>(U8());
      case 0xD1:
        return static_cast<int16_t>(U16());
      case 0xD2:
        return static_cast<int32_t>(U32());
      case 0xD3:
        return static_cast<int64_t>(U64());
      default:
        SkipMismatched();
        return 0;
    }
  }

  double ReadDouble() {
    int b = U8();
    if (b <= 0x7F) {
      return b;
    }
    if (b >= 0xE0) {
      return static_cast<int8_t>(b);
    }
    switch (b) {
      case 0xC0:
        return 0.0;
      case 0xC2:
        return 0.0;
      case 0xC3:
        return 1.0;
      case 0xCA:
        return Float32();
      case 0xCB:
        return Float64();
      case 0xCC:
        return U8();
      case 0xCD:
        return U16();
      case 0xCE:
        return static_cast<double>(U32());
      case 0xCF:
        return static_cast<double>(U64());
      case 0xD0:
        return static_cast<int8_t>(U8());
      case 0xD1:
        return static_cast<int16_t>(U16());
      case 0xD2:
        return static_cast<int32_t>(U32());
      case 0xD3:
        return static_cast<double>(static_cast<int64_t>(U64()));
      default:
        SkipMismatched();
        return 0.0;
    }
  }

  // A pose array the camera always sends with six values. Fewer than six
  // numbers, including a missing or malformed value, reads as six zeros, the
  // no-pose value. Longer arrays are kept as sent. Index access up to [5] is
  // always safe.
  std::vector<double> ReadPoseArray() {
    std::vector<double> out = ReadDoubleArray();
    if (out.size() < kPoseArrayLength) {
      return std::vector<double>(kPoseArrayLength, 0.0);
    }
    return out;
  }

  std::vector<double> ReadDoubleArray() {
    int n = ReadArrayHeader();
    std::vector<double> out;
    out.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; i++) {
      out.push_back(ReadDouble());
    }
    return out;
  }

  std::vector<std::vector<double>> ReadPointArray() {
    int n = ReadArrayHeader();
    std::vector<std::vector<double>> out;
    out.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; i++) {
      out.push_back(ReadDoubleArray());
    }
    return out;
  }

  void SkipValue() {
    // A corrupt envelope of nested container headers would otherwise recurse
    // once per byte and overflow the stack past Decode()'s never-throws catch
    if (++m_skipDepth > kMaxSkipDepth) {
      m_skipDepth--;
      throw std::runtime_error("msgpack nested deeper than " +
                               std::to_string(kMaxSkipDepth));
    }
    try {
      SkipValueInner();
    } catch (...) {
      m_skipDepth--;
      throw;
    }
    m_skipDepth--;
  }

 private:
  static constexpr int kMaxSkipDepth = 64;

  int U8() {
    RequireBytes(1);
    return m_buf[m_pos++];
  }

  int U16() {
    RequireBytes(2);
    int v = (m_buf[m_pos] << 8) | m_buf[m_pos + 1];
    m_pos += 2;
    return v;
  }

  uint32_t U32() {
    RequireBytes(4);
    uint32_t v = (static_cast<uint32_t>(m_buf[m_pos]) << 24) |
                 (static_cast<uint32_t>(m_buf[m_pos + 1]) << 16) |
                 (static_cast<uint32_t>(m_buf[m_pos + 2]) << 8) |
                 static_cast<uint32_t>(m_buf[m_pos + 3]);
    m_pos += 4;
    return v;
  }

  uint64_t U64() {
    RequireBytes(8);
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) {
      v = (v << 8) | m_buf[m_pos + static_cast<size_t>(i)];
    }
    m_pos += 8;
    return v;
  }

  float Float32() {
    uint32_t bits = U32();
    float out;
    static_assert(sizeof(out) == sizeof(bits));
    std::memcpy(&out, &bits, sizeof(out));
    return out;
  }

  double Float64() {
    uint64_t bits = U64();
    double out;
    static_assert(sizeof(out) == sizeof(bits));
    std::memcpy(&out, &bits, sizeof(out));
    return out;
  }

  void RequireBytes(size_t count) const {
    if (count > m_buf.size() - m_pos) {
      throw std::runtime_error("truncated msgpack value near byte " +
                               std::to_string(m_pos));
    }
  }

  // Rejects corrupt length prefixes before they drive a huge allocation or a
  // silent cursor desync (n items need >= n * minBytesPerItem remaining bytes)
  int CheckedCount(int count, int minBytesPerItem, const char* what) const {
    if (count < 0 ||
        static_cast<uint64_t>(count) * static_cast<uint64_t>(minBytesPerItem) >
            m_buf.size() - m_pos) {
      throw std::runtime_error("corrupt " + std::string{what} + " length " +
                               std::to_string(count) + " near byte " +
                               std::to_string(m_pos));
    }
    return count;
  }

  void SkipValueInner() {
    int b = U8();
    if (b <= 0x7F || b >= 0xE0) {
      return;  // fixint
    }
    if (b >= 0xA0 && b <= 0xBF) {
      SkipBytes(b & 0x1F);  // fixstr
      return;
    }
    if (b >= 0x90 && b <= 0x9F) {
      SkipValues(b & 0x0F);  // fixarray
      return;
    }
    if (b >= 0x80 && b <= 0x8F) {
      SkipValues(2 * (b & 0x0F));  // fixmap
      return;
    }
    switch (b) {
      case 0xC0:
      case 0xC2:
      case 0xC3:
        return;
      case 0xCC:
      case 0xD0:
        SkipBytes(1);
        return;
      case 0xCD:
      case 0xD1:
        SkipBytes(2);
        return;
      case 0xCE:
      case 0xD2:
      case 0xCA:
        SkipBytes(4);
        return;
      case 0xCF:
      case 0xD3:
      case 0xCB:
        SkipBytes(8);
        return;
      case 0xD9:
      case 0xC4:
        SkipBytes(U8());
        return;
      case 0xDA:
      case 0xC5:
        SkipBytes(U16());
        return;
      case 0xDB:
      case 0xC6:
        SkipBytes(static_cast<int64_t>(U32()));
        return;
      case 0xD4:
        SkipBytes(2);
        return;  // fixext1
      case 0xD5:
        SkipBytes(3);
        return;  // fixext2
      case 0xD6:
        SkipBytes(5);
        return;  // fixext4
      case 0xD7:
        SkipBytes(9);
        return;  // fixext8
      case 0xD8:
        SkipBytes(17);
        return;  // fixext16
      case 0xC7:
        SkipBytes(1 + static_cast<int64_t>(U8()));
        return;
      case 0xC8:
        SkipBytes(1 + static_cast<int64_t>(U16()));
        return;
      case 0xC9:
        SkipBytes(1 + static_cast<int64_t>(U32()));
        return;
      case 0xDC:
        SkipValues(CheckedCount(U16(), 1, "skipped array"));
        return;
      case 0xDD:
        SkipValues(CheckedCount(static_cast<int>(U32()), 1, "skipped array"));
        return;
      case 0xDE:
        SkipValues(2 * CheckedCount(U16(), 2, "skipped map"));
        return;
      case 0xDF:
        SkipValues(2 * CheckedCount(static_cast<int>(U32()), 2, "skipped map"));
        return;
      default:
        throw BadToken("value", b);
    }
  }

  void SkipBytes(int64_t count) {
    if (count < 0 || static_cast<uint64_t>(count) > m_buf.size() - m_pos) {
      throw std::runtime_error("truncated msgpack value near byte " +
                               std::to_string(m_pos));
    }
    m_pos += static_cast<size_t>(count);
  }

  void SkipValues(int count) {
    for (int i = 0; i < count; i++) {
      SkipValue();
    }
  }

  // Matches Java's (long) cast: NaN reads as 0, out-of-range values saturate.
  // A raw static_cast would be undefined behavior for those inputs.
  static int64_t FloatToInt64(double v) {
    if (std::isnan(v)) {
      return 0;
    }
    if (v >= 9223372036854775808.0) {
      return std::numeric_limits<int64_t>::max();
    }
    if (v <= -9223372036854775808.0) {
      return std::numeric_limits<int64_t>::min();
    }
    return static_cast<int64_t>(v);
  }

  std::runtime_error BadToken(const char* expected, int token) const {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "expected %s, got 0x%x at byte %zu",
                  expected, token, m_pos == 0 ? m_pos : m_pos - 1);
    return std::runtime_error(buf);
  }

  std::span<const uint8_t> m_buf;
  size_t m_pos = 0;
  int m_skipDepth = 0;
};

}  // namespace detail

}  // namespace limelight
