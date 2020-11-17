/* 
 * lattice_draw.cpp
 * 
 * Created on: Oct 25, 2018 12:16
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "state_lattice/lattice_draw.hpp"

using namespace rnav;

void LatticeViz::DrawTrajectoryPoints(CvCanvas &canvas, const std::vector<MotionState> &states, cv::Scalar ln_color, int32_t thickness)
{
    for (int32_t i = 0; i < states.size() - 1; ++i)
    {
        MotionState st1 = states[i];
        MotionState st2 = states[i + 1];

        CPoint pt1(st1.x, st1.y);
        CPoint pt2(st2.x, st2.y);
        canvas.DrawLine(pt1, pt2, ln_color, thickness);
    }
}

void LatticeViz::DrawMotionPrimitive(CvCanvas &canvas, MotionPrimitive mp, double step, cv::Scalar ln_color, int32_t thickness)
{
    std::vector<MotionState> states;

    for (double s = 0; s <= mp.GetLength(); s += step)
        states.push_back(mp.Evaluate(s, step / 5.0));

    // std::cout << "state size: " << states.size() << std::endl;

    LatticeViz::DrawTrajectoryPoints(canvas, states, ln_color, thickness);
}

void LatticeViz::DrawMotionPrimitive(CvCanvas &canvas, std::vector<MotionPrimitive> &mps, double step, cv::Scalar ln_color, int32_t thickness)
{
    for (auto &mp : mps)
        LatticeViz::DrawMotionPrimitive(canvas, mp, step, ln_color, thickness);
}

void LatticeViz::DrawStateLattice(CvCanvas &canvas, StateLattice sl, double step, cv::Scalar ln_color, int32_t thickness)
{
    std::vector<MotionState> states;

    for (double s = 0; s <= sl.GetLength(); s += step)
        states.push_back(sl.Evaluate(s, step / 5.0));

    // std::cout << "state size: " << states.size() << std::endl;

    LatticeViz::DrawTrajectoryPoints(canvas, states, ln_color, thickness);
}

void LatticeViz::DrawStateLattice(CvCanvas &canvas, std::vector<StateLattice> &sls, double step, cv::Scalar ln_color, int32_t thickness)
{
    for (auto &sl : sls)
        LatticeViz::DrawStateLattice(canvas, sl, step, ln_color, thickness);
}