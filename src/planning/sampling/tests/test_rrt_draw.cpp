#include <iostream>
#include <cstdint>

#include "sampling/rrt.hpp"
#include "sampling/space/realvector_space.hpp"
#include "sampling/steer/rv_straight_steer.hpp"

#include "sampling/rrt_draw.hpp"

using namespace robosw;

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
    // auto sstate = rvspace.CreateState({10, 5});
    // auto gstate = rvspace.CreateState({15, 8});

    std::cout << "start: " << *sstate << std::endl;
    std::cout << "goal: " << *gstate << std::endl;

    rrt.SetSteerFunction(RVStraightSteer<2>(&rvspace, 2));

    auto path =rrt.Search(sstate, gstate, 5000);

    /////////////////////////////////////////////

    CvCanvas canvas(50);
    canvas.Resize(-2, 22, -2, 12);
    canvas.SetMode(CvCanvas::DrawMode::GeometryInvertedY);

    canvas.DrawRectangle({0,0}, {20,10});
    RRTViz::DrawTree(canvas, rrt.GetTree());
    RRTViz::DrawStraightPath(canvas, path);

    canvas.Show();

    return 0;
}