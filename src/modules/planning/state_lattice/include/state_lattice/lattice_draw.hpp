/* 
 * lattice_draw.hpp
 * 
 * Created on: Oct 25, 2018 12:15
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LATTICE_DRAW_HPP
#define LATTICE_DRAW_HPP

#include "state_lattice/details/motion_state.hpp"
#include "state_lattice/motion_primitive.hpp"
#include "state_lattice/state_lattice.hpp"

#include "cvdraw/cvdraw.hpp"

namespace ivnav
{
namespace LatticeViz
{
void DrawTrajectoryPoints(CvCanvas &canvas, const std::vector<MotionState> &states, cv::Scalar ln_color = CvColors::orange_color, int32_t thickness = 1);

void DrawMotionPrimitive(CvCanvas &canvas, MotionPrimitive mp, double step = 0.1, cv::Scalar ln_color = CvColors::orange_color, int32_t thickness = 1);
void DrawMotionPrimitive(CvCanvas &canvas, std::vector<MotionPrimitive> &mps, double step = 0.1, cv::Scalar ln_color = CvColors::orange_color, int32_t thickness = 2);

void DrawStateLattice(CvCanvas &canvas, StateLattice sl, double step = 0.1, cv::Scalar ln_color = CvColors::orange_color, int32_t thickness = 1);
void DrawStateLattice(CvCanvas &canvas, std::vector<StateLattice> &sls, double step = 0.1, cv::Scalar ln_color = CvColors::orange_color, int32_t thickness = 1);
}; // namespace LatticeViz
} // namespace ivnav

#endif /* LATTICE_DRAW_HPP */
