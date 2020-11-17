#include <iostream>
#include <cstdint>

#include "state_lattice/details/lookup_table.hpp"

#define ENABLE_VIZ

#ifdef ENABLE_VIZ
#include "state_lattice/lattice_viz.hpp"
#endif

using namespace rnav;

int main()
{
    LookupTable table;
    table.GenerateLookupTable(true, "lookup_table");

    // LookupTable table("/home/rdu/lookup_table.20181026024917.data");

    auto mps = table.GetAllSeedPrimitives();
    // LightViz::ShowMotionPrimitive(mps, 0.1, 50, "Lattice Lookup", true);

    return 0;
}