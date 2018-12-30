#include <iostream>
#include <cstdint>

#include "sampling/details/space/realvector_space.hpp"
#include "sampling/rrt.hpp"
#include "stopwatch/stopwatch.h"

using namespace librav;

int main()
{
    RealVectorSpace<3> rvspace;

    rvspace.SetBound(0, 1, 1.5);
    rvspace.SetBound(1, 2, 3.5);
    rvspace.SetBound(2, 3, 4.5);

    rvspace.PrintInfo();

    // RealVectorSpace<2> rvspace2({{1, 1.5}, {1, 1.5}});
    // rvspace2.PrintInfo();

    // std::cout << "sampling states: " << std::endl;
    // stopwatch::StopWatch sw;
    // for (int i = 0; i < 10000; ++i)
    // {
    //     // rvspace.SampleUniform();
    //     std::cout << (*rvspace.SampleUniform()) << std::endl;
    // }
    // std::cout << "finished in " << sw.toc() << std::endl;

    RRT<RealVectorSpace<3>> rrt;

    return 0;
}