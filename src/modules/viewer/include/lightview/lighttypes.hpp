/*
 * lighttypes.hpp
 *
 * Created on: May 08, 2020 10:37
 * Description:
 *
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef LIGHTTYPES_HPP
#define LIGHTTYPES_HPP

namespace rav {
struct LtColor {
  LtColor(float _x, float _y, float _z, float _w)
      : x(_x), y(_y), z(_z), w(_w) {}

  float x = 0;
  float y = 0;
  float z = 0;
  float w = 0;
};
}  // namespace rav

#endif /* LIGHTTYPES_HPP */
