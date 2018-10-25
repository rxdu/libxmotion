#include <iostream>
#include <cstdint>

#include "state_lattice/details/point_kinematics.hpp"

using namespace librav;

int main()
{
    // PointKinematics model;
    // PointKinematics::Param param(1, 2, 1, 1, 1);
    // MotionState init(0, 0, 0, 0);
    // MotionState ss = model.Propagate(init, param);
    // std::cout << ss << std::endl;

    PointKinematics model(1, 0, 0, 0);
    MotionState init(0, 0, 0, 0);
    std::vector<MotionState> states = model.GenerateTrajectoryPoints(init, 5, 0.5);
    
    return 0;
}