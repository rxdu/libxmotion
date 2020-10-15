#include <iostream>
#include <cstdint>
#include <cmath>

#include "state_lattice/state_lattice.hpp"
#include "stopwatch/stopwatch.h"

#define ENABLE_VIZ

#ifdef ENABLE_VIZ
#include "state_lattice/lattice_viz.hpp"
#endif

using namespace ivnav;

int main()
{
    MotionState start(8, 2, M_PI / 6.0, 1);
    MotionState target(15, -3, M_PI / 5.0, 0.1);

    StateLattice lattice(start, target);

#ifdef ENABLE_VIZ
    if (lattice.IsValid())
        LightViz::ShowStateLattice(lattice, 0.1, 50);
#endif

    return 0;
}