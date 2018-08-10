/* 
 * polygon_viz.hpp
 * 
 * Created on: Aug 10, 2018 09:11
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef POLYGON_VIZ_HPP
#define POLYGON_VIZ_HPP

#include "polygon/polygon.hpp"

namespace librav
{
namespace LightViz
{
void ShowPolygon(const Polygon &polygon, int32_t pixel_per_unit = 10, std::string window_name = "Matrix Image", bool save_img = false);
}
} // namespace librav

#endif /* POLYGON_VIZ_HPP */
