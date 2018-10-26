#include <iostream>
#include <cstdint>
#include <cmath>

#include "state_lattice/state_lattice.hpp"

#include "ugvnav_viz/ugvnav_viz.hpp"
#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    MotionState start(2, 1, M_PI / 6.0, 1);
    MotionState target(15, 6, M_PI / 5.0, 0.1);

    StateLattice lattice(start, target);

    if (lattice.IsValid())
        LightViz::ShowStateLattice(lattice);

    return 0;
}