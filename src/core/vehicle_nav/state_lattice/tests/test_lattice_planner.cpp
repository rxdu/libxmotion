#include <memory>
#include <iostream>
#include <cstdint>

#include "state_lattice/lattice_manager.hpp"
#include "state_lattice/lattice_planner.hpp"

using namespace librav;

int main()
{
    std::shared_ptr<LatticeManager> lm = std::make_shared<LatticeManager>();

    lm->LoadPrimitivesFromFile("/home/rdu/mp.20180807061050.data");

    // auto new_base = lm->primitives_[35];
    // std::vector<MotionPrimitive> new_mps = lm->TransformAllPrimitives(lm->primitives_, new_base.GetFinalNode().x, new_base.GetFinalNode().y, new_base.GetFinalNode().theta);
    // lm->SavePrimitivesToFile(new_base.GetFinalNode(), new_mps, "mp_trans");

    LatticePlanner planner(lm);

    LatticeNode start(0, 0, 0);
    LatticeNode goal(20, 30, M_PI / 6.0);

    auto path = planner.Search(start, goal);

    return 0;
}