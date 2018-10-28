/* 
 * lattice_viz.cpp
 * 
 * Created on: Oct 25, 2018 11:35
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "ugvnav_viz/lattice_viz.hpp"

#include "ugvnav_viz/details/lattice_draw.hpp"

using namespace librav;
using namespace CvDraw;

void LightViz::ShowMotionStateTrajectory(const std::vector<MotionState> &states, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    CartesianCanvas canvas(pixel_per_unit);
    canvas.SetupCanvas(-8, 8, -5, 5);

    LatticeDraw ldraw(canvas);
    ldraw.DrawTrajectoryPoints(states);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void LightViz::ShowMotionPrimitive(MotionPrimitive mp, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    CartesianCanvas canvas(pixel_per_unit);
    canvas.SetupCanvas(0, 32, -8, 8);

    LatticeDraw ldraw(canvas);
    ldraw.DrawMotionPrimitive(mp, step);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void LightViz::ShowMotionPrimitive(std::vector<MotionPrimitive> &mps, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    CartesianCanvas canvas(pixel_per_unit);
    canvas.SetupCanvas(0, 32, -8, 8);

    LatticeDraw ldraw(canvas);
    ldraw.DrawMotionPrimitive(mps, step);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void LightViz::ShowStateLattice(StateLattice sl, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    CartesianCanvas canvas(pixel_per_unit);
    canvas.SetupCanvas(0, 32, -8, 8);

    LatticeDraw ldraw(canvas);
    ldraw.DrawStateLattice(sl, step);

    ShowImage(canvas.paint_area, window_name, save_img);
}

void LightViz::ShowStateLattice(std::vector<StateLattice> &sls, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    CartesianCanvas canvas(pixel_per_unit);
    // canvas.SetupCanvas(0, 32, -8, 8);
    canvas.SetupCanvas(10, 20, 50, 60);

    LatticeDraw ldraw(canvas);
    ldraw.DrawStateLattice(sls, step);

    ShowImage(canvas.paint_area, window_name, save_img);
}