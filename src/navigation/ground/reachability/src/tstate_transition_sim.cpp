/* 
 * tstate_transition_sim.cpp
 * 
 * Created on: Oct 30, 2018 05:41
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "reachability/details/tstate_transition_sim.hpp"
#include "reachability/details/tmodel_solver.hpp"

using namespace librav;

void TStateTransitionSim::SetupStateSpace(double smin, double smax, double vmin, double vmax, int32_t ssize, int32_t vsize)
{
    state_space_ = std::unique_ptr<TStateSpace>(new TStateSpace(smin, smax, vmin, vmax));
    state_space_->DiscretizeSpaceBySize(ssize, vsize);
}

void TStateTransitionSim::RunSim(double T)
{
    auto cells = state_space_->GetStateCellsByS(0);
    std::cout << "number of cells: " << cells.size() << std::endl;

    auto cell = cells.front();
    cell->PrintInfo();
    auto samples = cell->GetUniformSamples(10, 10);
    for (auto &sample : samples)
    {
        asc::state_t statef = propagator_.Propagate({sample.s, sample.v}, 0.5, 0, T, T / 10);
        std::cout << "final state: " << statef[0] << " , " << statef[1] << std::endl;
    }
    // for(auto cell : cells)
    // {

    // }
}