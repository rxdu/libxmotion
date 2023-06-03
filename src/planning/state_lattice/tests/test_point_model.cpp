#include <iostream>
#include <cstdint>

#include "state_lattice/details/point_kinematics.hpp"

#define ENABLE_VIZ

#ifdef ENABLE_VIZ
#include "state_lattice/lattice_viz.hpp"
#endif

using namespace xmotion;

int main()
{
    // PointKinematics model;
    // PointKinematics::Param param(1, 2, 1, 1, 1);
    // MotionState init(0, 0, 0, 0);
    // MotionState ss = model.Propagate(init, param);
    // std::cout << ss << std::endl;

    PointKinematics model(0.1, .1, .1, .2);
    MotionState init(0, 0, 0, 0);
    std::vector<MotionState> states = model.GenerateTrajectoryPoints(init, M_PI, 0.1, 0.01);

    // LightViz::ShowMotionStateTrajectory(states, 50);

    return 0;
}