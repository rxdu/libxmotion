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
#include "state_lattice/motion_primitive.hpp"
#include "state_lattice/state_lattice.hpp"

namespace rnav
{
namespace LightViz
{
void ShowMotionStateTrajectory(const std::vector<MotionState> &states, int32_t pixel_per_unit = 10, std::string window_name = "Lattice Image", bool save_img = false);

void ShowMotionPrimitive(MotionPrimitive mp, double step = 0.1, int32_t pixel_per_unit = 10, std::string window_name = "Lattice Image", bool save_img = false);
void ShowMotionPrimitive(std::vector<MotionPrimitive> &mps, double step = 0.1, int32_t pixel_per_unit = 10, std::string window_name = "Lattice Image", bool save_img = false);

void ShowStateLattice(StateLattice sl, double step = 0.1, int32_t pixel_per_unit = 10, std::string window_name = "Lattice Image", bool save_img = false);
void ShowStateLattice(std::vector<StateLattice> &sls, double step = 0.1, int32_t pixel_per_unit = 10, std::string window_name = "Lattice Image", bool save_img = false);
}; // namespace LightViz
} // namespace rnav

#endif /* LATTICE_VIZ_HPP */
