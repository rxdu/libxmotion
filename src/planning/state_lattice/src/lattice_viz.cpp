/* 
 * lattice_viz.cpp
 * 
 * Created on: Oct 25, 2018 11:35
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "state_lattice/lattice_viz.hpp"

#include "cvdraw/cvdraw.hpp"
#include "state_lattice/lattice_draw.hpp"

using namespace xmotion;

void LightViz::ShowMotionStateTrajectory(const std::vector<MotionState> &states, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    CvCanvas canvas(pixel_per_unit);
    canvas.Resize(-8, 8, -5, 5);

    LatticeViz::DrawTrajectoryPoints(canvas, states);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void LightViz::ShowMotionPrimitive(MotionPrimitive mp, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    CvCanvas canvas(pixel_per_unit);
    canvas.Resize(0, 32, -8, 8);

    LatticeViz::DrawMotionPrimitive(canvas, mp, step);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void LightViz::ShowMotionPrimitive(std::vector<MotionPrimitive> &mps, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    CvCanvas canvas(pixel_per_unit);
    canvas.Resize(0, 32, -8, 8);

    LatticeViz::DrawMotionPrimitive(canvas, mps, step);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void LightViz::ShowStateLattice(StateLattice sl, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    CvCanvas canvas(pixel_per_unit);
    canvas.Resize(0, 32, -8, 8);

    LatticeViz::DrawStateLattice(canvas, sl, step);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void LightViz::ShowStateLattice(std::vector<StateLattice> &sls, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    CvCanvas canvas(pixel_per_unit);
    // canvas.Resize(0, 32, -8, 8);
    canvas.Resize(10, 20, 50, 60);

    LatticeViz::DrawStateLattice(canvas, sls, step);

    CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}