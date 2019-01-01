#include <iostream>
#include <cstdint>

#define SHOW_TREE_GROWTH

#include "sampling/rrt.hpp"
#include "sampling/space/realvector_space.hpp"
#include "sampling/steer/rv_straight_steer.hpp"
#include "sampling/validity/rv_polygon_validity_checker.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    RealVectorSpace<2> rvspace;

    rvspace.SetBound(0, 0, 20);
    rvspace.SetBound(1, 0, 10);

    rvspace.PrintInfo();

    RRT<RealVectorSpace<2>> rrt(&rvspace);

    // auto sstate = rvspace.SampleUniform();
    // auto gstate = rvspace.SampleUniform();
    auto sstate = rvspace.CreateState({0, 0});
    auto gstate = rvspace.CreateState({20, 10});

    std::cout << "start: " << *sstate << std::endl;
    std::cout << "goal: " << *gstate << std::endl;

    rrt.SetSteerFunction(RVStraightSteer<2>(&rvspace, 1));

    RVPolygonValidityChecker checker;
    rrt.SetStateValidityChecker(checker);
    rrt.SetPathValidityChecker(checker);

    rrt.Search(sstate, gstate, 5000);

    return 0;
}