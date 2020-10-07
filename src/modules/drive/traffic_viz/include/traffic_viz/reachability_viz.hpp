/* 
 * reachability_viz.hpp
 * 
 * Created on: Oct 30, 2018 09:11
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef REACHABILITY_VIZ_HPP
#define REACHABILITY_VIZ_HPP

#include <string>
#include <cstdint>

#include "reachability/tstate_space.hpp"

namespace autodrive
{
namespace LightViz
{
void ShowTStateSpace(TStateSpace &space, int32_t pixel_per_unit = 100, std::string window_name = "Reachability Image", bool save_img = false);
};
} // namespace autodrive

#endif /* REACHABILITY_VIZ_HPP */
