#include <iostream>
#include <cstdint>

#include "state_lattice/lattice_generator.hpp"

using namespace librav;

int main()
{
    LatticeGenerator gen;

    gen.SetInitialState(0, 0, 12, 0);
    gen.GenerateControlSet();
    gen.RunSim(0,1.0,0.1);

    return 0;
}