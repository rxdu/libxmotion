/*
 * types.hpp
 *
 * Created on: Dec 14, 2021 09:11
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#ifndef TYPES_HPP
#define TYPES_HPP

#include <cstdint>
#include <chrono>

namespace xmotion {
template <typename T>
struct vector3_t {
  T x;
  T y;
  T z;
};

using Vector3f = vector3_t<float>;
using Vector3d = vector3_t<double>;

template <typename T>
struct vector4_t {
  T x;
  T y;
  T z;
  T w;
};

using Vector4f = vector4_t<float>;
using Vector4d = vector4_t<double>;

using RSClock = std::chrono::steady_clock;
using RSTimePoint = std::chrono::time_point<RSClock>;

using RSEnumBaseType = uint32_t;
}  // namespace xmotion

#endif /* TYPES_HPP */
