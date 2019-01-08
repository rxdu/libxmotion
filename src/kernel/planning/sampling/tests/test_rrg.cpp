#include <iostream>
#include <cstdint>

#define SHOW_TREE_GROWTH

#include "sampling/rrg.hpp"
#include "sampling/space/realvector_space.hpp"
#include "sampling/steer/rv_straight_steer.hpp"
#include "sampling/validity/rv_polygon_validity_checker.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    constexpr int32_t N = 2;

    RealVectorSpace<N> rvspace;

    rvspace.SetBound(0, 0, 20);
    rvspace.SetBound(1, 0, 10);

    rvspace.PrintInfo();

    RRG<RealVectorSpace<N>> rrg(&rvspace);

    // auto sstate = rvspace.SampleUniform();
    // auto gstate = rvspace.SampleUniform();
    auto sstate = rvspace.CreateState({0, 0});
    auto gstate = rvspace.CreateState({20, 10});

    std::cout << "start: " << *sstate << std::endl;
    std::cout << "goal: " << *gstate << std::endl;

    rrg.SetExtendStepSize(5.0);
    rrg.SetSteerFunction(RVStraightSteer<N>(&rvspace, 5.0));

    RVPolygonValidityChecker checker;
    rrg.SetStateValidityChecker(checker);
    rrg.SetPathValidityChecker(checker);

    rrg.SetOptimizationConstant(50);

    auto path = rrg.Search(sstate, gstate, 200);

    double distance = 0;
    for (int i = 0; i < path.size() - 1; ++i)
        distance += rvspace.EvaluateDistance(path[i], path[i + 1]);
    std::cout << "path length: " << distance << std::endl;

    return 0;
}