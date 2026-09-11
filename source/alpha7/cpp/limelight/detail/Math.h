// Copyright (c) 2026 Limelight Vision. All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <cmath>
namespace limelight {

namespace detail {

constexpr double ClampArg(double value, double min, double max,
                          double nanFallback) {
  if (std::isnan(value)) {
    return nanFallback;
  }
  return std::max(min, std::min(max, value));
}

constexpr double ClampArg(double value, double min, double max) {
  return ClampArg(value, min, max, min);
}

}  // namespace detail

}  // namespace limelight
