#include <iostream>
#include <cstdint>

#include "state_lattice/lattice_generator.hpp"

using namespace librav;

int main()
{
    LatticeGenerator gen;

    gen.SetInitialState(0, 0, 10, 0);
    gen.GenerateControlSet();
    gen.RunSim(0,1.0,0.2);

    return 0;
}