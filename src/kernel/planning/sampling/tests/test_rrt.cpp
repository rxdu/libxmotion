#include <iostream>
#include <cstdint>

#include "sampling/rrt.hpp"
#include "sampling/space/realvector_space.hpp"
#include "sampling/steer/rv_straight_steer.hpp"

#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    RealVectorSpace<3> rvspace;

    rvspace.SetBound(0, 1, 25);
    rvspace.SetBound(0, 2, 15);

    rvspace.PrintInfo();

    // std::cout << "sampling states: " << std::endl;
    // stopwatch::StopWatch sw;
    // for (int i = 0; i < 10000; ++i)
    // {
    //     // rvspace.SampleUniform();
    //     std::cout << (*rvspace.SampleUniform()) << std::endl;
    // }
    // std::cout << "finished in " << sw.toc() << std::endl;

    RRT<RealVectorSpace<3>> rrt(&rvspace);

    auto sstate = rvspace.SampleUniform();
    auto gstate = rvspace.SampleUniform();

    rrt.SetSteerFunction(RVStraightSteer<3>(&rvspace, 1));

    rrt.Search(sstate, gstate, 5000);

    return 0;
}