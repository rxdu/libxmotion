/* 
 * lattice_viz.hpp
 * 
 * Created on: Oct 25, 2018 10:37
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LATTICE_VIZ_HPP
#define LATTICE_VIZ_HPP

#include <vector>

#include "state_lattice/details/motion_state.hpp"

namespace librav
{
namespace LightViz
{
void ShowMotionStateTrajectory(const std::vector<MotionState> &states, int32_t pixel_per_unit = 10, std::string window_name = "Lattice Image", bool save_img = false);
};
} // namespace librav

#endif /* LATTICE_VIZ_HPP */
