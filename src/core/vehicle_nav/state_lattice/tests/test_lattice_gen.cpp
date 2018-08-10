#include <iostream>
#include <cstdint>

#include "state_lattice/lattice_generator.hpp"

using namespace librav;

int main()
{
    LatticeGenerator gen;

    // ~25mph : 11.17
    // ~20mph : 8.9
    // ~15mph : 6.7
    gen.SetInitialState(0, 0, 8.9, 0);
    gen.GenerateControlSet();
    gen.RunSim(0,1.0,0.2);

    return 0;
}