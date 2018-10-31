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
    state_space_ = std::make_shared<TStateSpace>(smin, smax, vmin, vmax);
    state_space_->DiscretizeSpaceBySize(ssize, vsize);
}

void TStateTransitionSim::RunSim(double T)
{
    // init statistics
    auto all_cells = state_space_->GetAllStateCells();
    for (auto cell_row : all_cells)
        for (auto cell : cell_row)
            for (int i = 0; i < control_set_.size(); ++i)
            {
                cell->occupancy_stats[i] = 0;
                cell->occupancy_probability[i] = 0.0;
            }

    state_space_->control_set_size_ = 6;
    state_space_->sim_number_per_ctrl_ = 20 * 30;

    // calculate Psi by running simulations
    auto cells = state_space_->GetStateCellsByS(0);
    std::cout << "number of cells: " << cells.size() << std::endl;
    auto cell = cells.front();
    // for(auto cell : cells)
    // {
    cell->PrintInfo();
    auto samples = cell->GetUniformSamples(20, 30);

    // int i = 3;
    for (int i = 0; i < control_set_.size(); ++i)
    {
        for (auto &sample : samples)
        {
            asc::state_t statef = propagator_.Propagate({sample.s, sample.v}, control_set_(i), 0, T, T / 10);
            // std::cout << "control: " << control_set_(i) << " , final state: " << statef[0] << " , " << statef[1] << std::endl;
            
            auto cell = state_space_->GetStateCell(statef[0], statef[1]);
            if (cell != nullptr)
            {
                cell->occupancy_stats[i]++;
            }
            else
            {
                std::cerr << "Out-of-bound error: consider increasing state space!" << std::endl;
                std::cout << " -- (s,v)_0 : " << sample.s << " , " << sample.v << ", (s,v)_f : " << statef[0] << " , " << statef[1] << " , control : " << control_set_(i) << std::endl;
            }
        }
    }

    // }
}