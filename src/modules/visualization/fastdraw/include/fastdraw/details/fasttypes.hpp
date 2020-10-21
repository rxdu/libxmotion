/*
 * fasttypes.hpp
 *
 * Created on: Oct 21, 2020 22:32
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef FASTTYPES_HPP
#define FASTTYPES_HPP

#include <cstdint>
#include <iostream>

namespace ivnav {
template <typename T>
struct FPoint2 {
  FPoint2(T _x = 0, T _y = 0) : x(_x), y(_y) {}

  T x;
  T y;

  friend std::ostream &operator<<(std::ostream &os, const FPoint2 &pt) {
    os << "point (x, y): " << pt.x << " , " << pt.y;
    return os;
  }
};

template <typename T>
struct FPoint3 {
  FPoint3(T _x = 0, T _y = 0, T _z = 0) : x(_x), y(_y), z(_z) {}

  T x;
  T y;
  T z;

  friend std::ostream &operator<<(std::ostream &os, const FPoint3 &pt) {
    os << "point (x, y): " << pt.x << " , " << pt.y << " , " << pt.z;
    return os;
  }
};

template <typename T>
struct FPoint4 {
  FPoint4(T _x = 0, T _y = 0, T _z = 0) : x(_x), y(_y), z(_z) {}

  T x;
  T y;
  T z;
  T w;

  friend std::ostream &operator<<(std::ostream &os, const FPoint4 &pt) {
    os << "point (x, y): " << pt.x << " , " << pt.y << " , " << pt.z << " , "
       << pt.w;
    return os;
  }
};

using FPoint2i = FPoint2<uint32_t>;
using FPoint2f = FPoint2<float>;
using FPoint2d = FPoint2<double>;

using FPoint3i = FPoint3<uint32_t>;
using FPoint3f = FPoint3<float>;
using FPoint3d = FPoint3<double>;

using FPoint4i = FPoint4<uint32_t>;
using FPoint4f = FPoint4<float>;
using FPoint4d = FPoint4<double>;

using Quaternionf = FPoint4f;
using Quaterniond = FPoint4d;

struct FColor {
  FColor(uint8_t _r, uint8_t _g, uint8_t _b, uint8_t _a)
      : r(_r), g(_g), b(_b), a(_a) {}

  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t a;
};

const FColor FCOLOR_LIGHTGRAY(00, 200, 200, 255);
const FColor FCOLOR_RAYWHITE(245, 245, 245, 255);

}  // namespace ivnav

#endif /* FASTTYPES_HPP */
