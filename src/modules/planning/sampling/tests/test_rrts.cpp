#include <iostream>
#include <cstdint>

#define SHOW_TREE_GROWTH

#include "sampling/rrt_star.hpp"
#include "sampling/space/realvector_space.hpp"
#include "sampling/steer/rv_straight_steer.hpp"
#include "sampling/validity/rv_polygon_validity_checker.hpp"

#include "stopwatch/stopwatch.h"

using namespace autodrive;

int main()
{
    constexpr int32_t N = 2;

    RealVectorSpace<N> rvspace;

    rvspace.SetBound(0, 0, 20);
    rvspace.SetBound(1, 0, 10);

    rvspace.PrintInfo();

    RRTStar<RealVectorSpace<N>> rrts(&rvspace);

    // auto sstate = rvspace.SampleUniform();
    // auto gstate = rvspace.SampleUniform();
    auto sstate = rvspace.CreateState({0, 0});
    auto gstate = rvspace.CreateState({20, 10});

    std::cout << "start: " << *sstate << std::endl;
    std::cout << "goal: " << *gstate << std::endl;

    rrts.SetExtendStepSize(5.0);
    rrts.SetSteerFunction(RVStraightSteer<N>(&rvspace, 5.0));

    RVPolygonValidityChecker checker;
    rrts.SetStateValidityChecker(checker);
    rrts.SetPathValidityChecker(checker);

    rrts.SetOptimizationConstant(50);

    auto path = rrts.Search(sstate, gstate, 500);

    double distance = 0;
    for (int i = 0; i < path.size() - 1; ++i)
        distance += rvspace.EvaluateDistance(path[i], path[i + 1]);
    std::cout << "path length: " << distance << std::endl;

    return 0;
}