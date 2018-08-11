#include <iostream>
#include <cstdint>

#include "state_lattice/lattice_manager.hpp"

using namespace librav;

int main()
{
    LatticeManager lm;

    lm.LoadPrimitivesFromFile("/home/rdu/mp.20180807061050.data");
    auto new_base = lm.primitives_[35];
    std::vector<MotionPrimitive> new_mps = lm.TransformAllPrimitives(lm.primitives_, new_base.GetFinalNode().x, new_base.GetFinalNode().y, new_base.GetFinalNode().theta);
    lm.SavePrimitivesToFile(new_mps, "mp_trans");

    return 0;
}