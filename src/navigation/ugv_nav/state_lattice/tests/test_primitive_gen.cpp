#include <iostream>
#include <cstdint>
#include <cmath>

#include "state_lattice/primitive_generator.hpp"

using namespace librav;

int main()
{
    PrimitiveGenerator gen;

    MotionState start(0, 0, 0, 1);
    MotionState target(1.0, 1.0, M_PI/2.0, 1);

    PointKinematics::Param init_p(1.2, 0.0, 0, 0, 1.5);

    gen.Calculate(start, target, init_p);

    return 0;
}