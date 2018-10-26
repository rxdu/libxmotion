#include <iostream>
#include <cstdint>
#include <cmath>

#include "state_lattice/primitive_generator.hpp"
#include "ugvnav_viz/ugvnav_viz.hpp"

using namespace librav;

int main()
{
    PrimitiveGenerator gen;

    // MotionState start(0, 0, 0, 1);
    // MotionState target(1.0, 1.0, M_PI/2.0, 1);
    // PointKinematics::Param init_p(1.0, 0.0, 1.0, 0, 1.45);

    MotionState start(0, 0, 0, 1);
    MotionState target(1.0, 1.0, M_PI / 2.0, 1);
    PointKinematics::Param init_p(1.1, 0.1, 1.2, 0, 1.45);

    auto mp = gen.Calculate(start, target, init_p);

    LightViz::ShowMotionPrimitive(mp, 0.1, 20);

    return 0;
}