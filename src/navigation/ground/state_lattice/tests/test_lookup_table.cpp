#include <iostream>
#include <cstdint>

#include "state_lattice/details/lookup_table.hpp"

#define ENABLE_VIZ

#ifdef ENABLE_VIZ
#include "lightviz/navviz.hpp"
#endif

using namespace librav;

int main()
{
    LookupTable table;
    table.GenerateLookupTable(true, "lookup_table");

    // LookupTable table("/home/rdu/lookup_table.20181026024917.data");

    auto mps = table.GetAllSeedPrimitives();
    // LightViz::ShowMotionPrimitive(mps, 0.1, 50, "Lattice Lookup", true);

    return 0;
}