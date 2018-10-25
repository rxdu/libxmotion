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
#include "lightviz/details/cartesian_canvas.hpp"

namespace librav
{
class LatticeDraw
{
  public:
    LatticeDraw(CartesianCanvas &canvas) : canvas_(canvas){};

    void DrawTrajectoryPoints(const std::vector<MotionState> &states);

  private:
    CartesianCanvas &canvas_;
};
} // namespace librav

#endif /* LATTICE_DRAW_HPP */
