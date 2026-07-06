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

#include "xmnav/state_lattice/details/motion_state.hpp"
#include "xmnav/state_lattice/motion_primitive.hpp"
#include "xmnav/state_lattice/state_lattice.hpp"

#include "cvdraw/cvdraw.hpp"

namespace xmotion
{
namespace LatticeViz
{
void DrawTrajectoryPoints(quickviz::CvCanvas &canvas, const std::vector<MotionState> &states, cv::Scalar ln_color = quickviz::CvColors::orange_color, int32_t thickness = 1);

void DrawMotionPrimitive(quickviz::CvCanvas &canvas, MotionPrimitive mp, double step = 0.1, cv::Scalar ln_color = quickviz::CvColors::orange_color, int32_t thickness = 1);
void DrawMotionPrimitive(quickviz::CvCanvas &canvas, std::vector<MotionPrimitive> &mps, double step = 0.1, cv::Scalar ln_color = quickviz::CvColors::orange_color, int32_t thickness = 2);

void DrawStateLattice(quickviz::CvCanvas &canvas, StateLattice sl, double step = 0.1, cv::Scalar ln_color = quickviz::CvColors::orange_color, int32_t thickness = 1);
void DrawStateLattice(quickviz::CvCanvas &canvas, std::vector<StateLattice> &sls, double step = 0.1, cv::Scalar ln_color = quickviz::CvColors::orange_color, int32_t thickness = 1);
}; // namespace LatticeViz
} // namespace xmotion

#endif /* LATTICE_DRAW_HPP */
