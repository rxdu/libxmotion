/* 
 * lattice_viz.cpp
 * 
 * Created on: Oct 25, 2018 11:35
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "xmnav/viz/lattice_viz.hpp"

#include "image/image.hpp"
#include "xmnav/viz/lattice_draw.hpp"

using namespace xmotion;

void LightViz::ShowMotionStateTrajectory(const std::vector<MotionState> &states, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    quickviz::CvCanvas canvas(pixel_per_unit);
    canvas.Resize(-8, 8, -5, 5);

    LatticeViz::DrawTrajectoryPoints(canvas, states);

    quickviz::CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void LightViz::ShowMotionPrimitive(MotionPrimitive mp, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    quickviz::CvCanvas canvas(pixel_per_unit);
    canvas.Resize(0, 32, -8, 8);

    LatticeViz::DrawMotionPrimitive(canvas, mp, step);

    quickviz::CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void LightViz::ShowMotionPrimitive(std::vector<MotionPrimitive> &mps, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    quickviz::CvCanvas canvas(pixel_per_unit);
    canvas.Resize(0, 32, -8, 8);

    LatticeViz::DrawMotionPrimitive(canvas, mps, step);

    quickviz::CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void LightViz::ShowStateLattice(StateLattice sl, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    quickviz::CvCanvas canvas(pixel_per_unit);
    canvas.Resize(0, 32, -8, 8);

    LatticeViz::DrawStateLattice(canvas, sl, step);

    quickviz::CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}

void LightViz::ShowStateLattice(std::vector<StateLattice> &sls, double step, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    quickviz::CvCanvas canvas(pixel_per_unit);
    // canvas.Resize(0, 32, -8, 8);
    canvas.Resize(10, 20, 50, 60);

    LatticeViz::DrawStateLattice(canvas, sls, step);

    quickviz::CvIO::ShowImage(canvas.GetPaintArea(), window_name, save_img);
}