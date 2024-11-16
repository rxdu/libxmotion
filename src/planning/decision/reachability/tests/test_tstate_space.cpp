#include <iostream>

#include "reachability/tstate_space.hpp"
#include <eigen3/Eigen/Dense>

using namespace xmotion;

int main()
{
    TStateSpace tspace(0, 50, 0, 20);
    tspace.DiscretizeSpace(5, 2);

    auto cell = tspace.GetStateCell(4, 1);
    cell->PrintInfo();

    auto samples = cell->GetUniformSamples(4, 4);
    for (auto &smp : samples)
        std::cout << smp << std::endl;
    std::cout << "sample size: " << samples.size() << std::endl;

    return 0;
}