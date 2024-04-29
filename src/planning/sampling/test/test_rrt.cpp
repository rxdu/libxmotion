#include <iostream>
#include <cstdint>

#define SHOW_TREE_GROWTH

#include "sampling/rrt.hpp"
#include "sampling/space/realvector_space.hpp"
#include "sampling/steer/rv_straight_steer.hpp"
#include "sampling/validity/rv_polygon_validity_checker.hpp"

#include "time/stopwatch.hpp"

using namespace xmotion;

int main()
{
    RealVectorSpace<2> rvspace;

    rvspace.SetBound(0, 0, 20);
    rvspace.SetBound(1, 0, 10);

    rvspace.PrintInfo();

    // RRT<RealVectorSpace<2>> rrt(&rvspace);
    RRT<RealVectorSpace<2>, KdTree<RealVectorSpace<2>>> rrt(&rvspace);

    // auto sstate = rvspace.SampleUniform();
    // auto gstate = rvspace.SampleUniform();
    auto sstate = rvspace.CreateState({0, 0});
    auto gstate = rvspace.CreateState({20, 10});

    std::cout << "start: " << *sstate << std::endl;
    std::cout << "goal: " << *gstate << std::endl;

    rrt.SetExtendStepSize(1.0);
    rrt.SetSteerFunction(RVStraightSteer<2>(&rvspace, 1.0));

    // RVPolygonValidityChecker checker;
    // Polygon collision{{5, 3}, {15, 3}, {15, 7}, {5, 7}};
    // checker.AddCollisionPolygon(collision);
    // rrt.SetStateValidityChecker(checker);
    // rrt.SetPathValidityChecker(checker);

    auto path = rrt.Search(sstate, gstate, 5000);

    double distance = 0;
    for (int i = 0; i < path.size() - 1; ++i)
        distance += rvspace.EvaluateDistance(path[i], path[i + 1]);
    std::cout << "path length: " << distance << std::endl;

    return 0;
}