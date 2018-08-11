/* 
 * distribution_viz.hpp
 * 
 * Created on: Aug 11, 2018 09:44
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef DISTRIBUTION_VIZ_HPP
#define DISTRIBUTION_VIZ_HPP

#include <vector>

#include "geometry/polygon.hpp"

namespace librav
{
namespace LightViz
{
void ShowPathLaneInField(const std::vector<Polyline> &bounds, const std::vector<Polyline> &centers,
                         std::vector<Polyline> &path,
                         double cx, double cy, std::function<double(double, double)> dist_fun,
                         bool show_wp = true, int32_t pixel_per_unit = 10, std::string window_name = "Lane Polyline Image", bool save_img = false);
}
} // namespace librav

#endif /* DISTRIBUTION_VIZ_HPP */
