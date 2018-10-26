/* 
 * lattice_draw.cpp
 * 
 * Created on: Oct 25, 2018 12:16
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "ugvnav_viz/details/lattice_draw.hpp"

using namespace librav;

void LatticeDraw::DrawTrajectoryPoints(const std::vector<MotionState> &states, cv::Scalar ln_color, int32_t ln_width)
{
    for (int32_t i = 0; i < states.size() - 1; ++i)
    {
        MotionState st1 = states[i];
        MotionState st2 = states[i + 1];

        auto pt1 = canvas_.ConvertCartisianToPixel(st1.x, st1.y);
        auto pt2 = canvas_.ConvertCartisianToPixel(st2.x, st2.y);
        LightViz::DrawLine(canvas_.paint_area, cv::Point(pt1.x, pt1.y), cv::Point(pt2.x, pt2.y), ln_color, ln_width);
    }
}

void LatticeDraw::DrawMotionPrimitive(MotionPrimitive mp, double step, cv::Scalar ln_color, int32_t ln_width)
{
    std::vector<MotionState> states;

    for (double s = 0; s <= mp.sf_; s += step)
        states.push_back(mp.Evaluate(s, step / 5.0));

    // std::cout << "state size: " << states.size() << std::endl;

    DrawTrajectoryPoints(states, ln_color, ln_width);
}

void LatticeDraw::DrawMotionPrimitive(std::vector<MotionPrimitive> &mps, double step, cv::Scalar ln_color, int32_t ln_width)
{
    for (auto &mp : mps)
        DrawMotionPrimitive(mp, step, ln_color, ln_width);
}