#include <iostream>
#include <cstdint>
#include <cmath>

#include "state_lattice/primitive_generator.hpp"
#include "ugvnav_viz/ugvnav_viz.hpp"

using namespace librav;

int main()
{
    PrimitiveGenerator gen;

    // MotionState start(0, 0, 0, 0);
    // MotionState target(1.0, 0.0, 0.0, 0);
    // PointKinematics::Param init_p(0.0, 0.0, 0.0, 0, 1.0);

    MotionState start(0, 0, 0, 0);
    MotionState target(5.0, 0.0, 0.0, 0);
    PointKinematics::Param init_p(0.0, 0.0, 0.0, 0, 5.0);

    // MotionState start(0, 0, 0, 1);
    // MotionState target(1.0, 1.0, M_PI / 2.0, 1);
    // PointKinematics::Param init_p(1.1, 0.1, 1.2, 0, 1.45);

    MotionPrimitive mp;
    bool result = gen.Calculate(start, target, init_p, mp);
    if (result)
        LightViz::ShowMotionPrimitive(mp, 0.1, 20);
    else
        std::cout << "--> Failed to find a path <--" << std::endl;

    return 0;
}