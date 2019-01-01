#include <iostream>
#include <cstdint>

#include "sampling/rrt.hpp"
#include "sampling/space/realvector_space.hpp"
#include "sampling/steer/rv_straight_steer.hpp"

#include "lightviz/details/rrt_draw.hpp"

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
    auto sstate = rvspace.CreateState({10, 5});
    // auto gstate = rvspace.CreateState({20, 10});
    auto gstate = rvspace.CreateState({15, 8});

    std::cout << "start: " << *sstate << std::endl;
    std::cout << "goal: " << *gstate << std::endl;

    rrt.SetSteerFunction(RVStraightSteer<2>(&rvspace, 2));

    rrt.Search(sstate, gstate, 5000);

    /////////////////////////////////////////////

    CartesianCanvas canvas(50);
    canvas.SetupCanvas(-2, 22, -2, 12);

    RRTDraw rrtdraw(canvas);
    rrtdraw.DrawTree(&rrt.tree_);
    // gdraw.DrawCurvilinearGrid(grid, 0.1);
    CvDraw::ShowImage(canvas.paint_area, "rrt");

    return 0;
}