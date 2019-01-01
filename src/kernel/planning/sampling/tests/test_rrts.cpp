#include <iostream>
#include <cstdint>

#include "sampling/rrt_star.hpp"
#include "sampling/space/realvector_space.hpp"
#include "sampling/steer/rv_straight_steer.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    RealVectorSpace<2> rvspace;

    rvspace.SetBound(0, 0, 20);
    rvspace.SetBound(1, 0, 10);

    rvspace.PrintInfo();

    // std::cout << "sampling states: " << std::endl;
    // stopwatch::StopWatch sw;
    // for (int i = 0; i < 10000; ++i)
    // {
    //     // rvspace.SampleUniform();
    //     std::cout << (*rvspace.SampleUniform()) << std::endl;
    // }
    // std::cout << "finished in " << sw.toc() << std::endl;

    RRTStar<RealVectorSpace<2>> rrt(&rvspace);

    // auto sstate = rvspace.SampleUniform();
    // auto gstate = rvspace.SampleUniform();
    // auto sstate = rvspace.CreateState({10, 5});
    // auto gstate = rvspace.CreateState({15, 8});
    auto sstate = rvspace.CreateState({0, 0});
    auto gstate = rvspace.CreateState({20, 10});

    std::cout << "start: " << *sstate << std::endl;
    std::cout << "goal: " << *gstate << std::endl;

    rrt.SetSteerFunction(RVStraightSteer<2>(&rvspace, 1));

    rrt.Search(sstate, gstate, 5000);

    return 0;
}