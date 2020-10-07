#include <iostream>
#include <cstdint>
#include <cmath>

#include "state_lattice/state_lattice.hpp"

#include "state_lattice/lattice_viz.hpp"
#include "stopwatch/stopwatch.h"

using namespace autodrive;

int main()
{
    MotionState start(8, 2, M_PI / 6.0, 1);
    MotionState target(15, -3, M_PI / 5.0, 0.1);

    StateLattice lattice(start, target);

    if (lattice.IsValid())
        LightViz::ShowStateLattice(lattice, 0.1, 50);

    return 0;
}