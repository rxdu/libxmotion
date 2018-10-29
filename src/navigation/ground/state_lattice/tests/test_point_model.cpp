#include <iostream>
#include <cstdint>

#include "state_lattice/details/point_kinematics.hpp"
#include "ugvnav_viz/ugvnav_viz.hpp"

using namespace librav;

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

    LightViz::ShowMotionStateTrajectory(states, 50);

    return 0;
}