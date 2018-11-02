/* 
 * tstate_transition_sim.cpp
 * 
 * Created on: Oct 30, 2018 05:41
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "reachability/details/tstate_transition_sim.hpp"

#include <unordered_map>

#include "reachability/details/tmodel_solver.hpp"

using namespace librav;

void TStateTransitionSim::SetupStateSpace(double smin, double smax, double vmin, double vmax, int32_t ssize, int32_t vsize)
{
    state_space_ = std::make_shared<TStateSpace>(smin, smax, vmin, vmax);
    state_space_->DiscretizeSpaceBySize(ssize, vsize);
}

Eigen::MatrixXd TStateTransitionSim::RunSim(double T)
{
    /*** init statistics ***/
    auto all_cells = state_space_->GetAllStateCells();

    // init stats to be zero
    for (auto cell_col : all_cells)
        for (auto cell : cell_col)
            cell->occupancy_stats = 0;

    const int32_t cell_sample_s = 20;
    const int32_t cell_sample_v = 30;

    state_space_->control_set_size_ = control_set_.size();
    state_space_->sim_number_per_ctrl_ = cell_sample_s * cell_sample_v;

    /*** create a very large matrix to hold Psi ***/
    Eigen::MatrixXd Psi;

    int32_t d_size = state_space_->GetStateNumber();
    int32_t c_size = control_set_.size();

    int32_t psi_block_size = d_size * c_size;
    Psi.resize(psi_block_size, psi_block_size);
    Psi.setZero(psi_block_size, psi_block_size);

    /*** calculate Psi by running simulations ***/
    for (int c = 0; c < c_size; ++c)
    {
        for (auto &cell_s_col : all_cells)
        {
            for (auto cell : cell_s_col)
            {
                // cell->PrintInfo();
                auto samples = cell->GetUniformSamples(cell_sample_s, cell_sample_v);

                std::unordered_map<int32_t, TStateSpace::StateCell *> nonezero_cells;

                // perform simulation with cell and control
                for (auto &sample : samples)
                {
                    asc::state_t statef = propagator_.Propagate({sample.s, sample.v}, control_set_(c), 0, T, T / 10);
                    // std::cout << "control: " << control_set_(i) << " , final state: " << statef[0] << " , " << statef[1] << std::endl;

                    auto final_cell = state_space_->GetStateCell(statef[0], statef[1]);
                    if (final_cell != nullptr)
                    {
                        final_cell->occupancy_stats++;
                        // std::cout << "id: " << final_cell->id << " , count: " << final_cell->occupancy_stats << std::endl;
                        nonezero_cells[final_cell->id] = final_cell;
                    }
                    // else
                    // {
                    //     std::cerr << "Out-of-bound error: consider increasing state space!" << std::endl;
                    //     std::cout << " -- (s,v)_0 : " << sample.s << " , " << sample.v << ", (s,v)_f : " << statef[0] << " , " << statef[1] << " , control : " << control_set_(c) << std::endl;
                    // }
                }

                // calculate and store probability, reset stats variables
                for (auto tc : nonezero_cells)
                {
                    double probability = tc.second->occupancy_stats / static_cast<double>(samples.size());
                    Psi(c_size * tc.second->id + c, c_size * cell->id + c) = probability;
                    // std::cout << "row: " << c_size * tc.second->id << " , " << tc.second->id << " , col: " << c_size * cell->id << " , probability = " << probability << std::endl;
                    
                    // reset variable for next simulation
                    tc.second->occupancy_stats = 0;
                }
            }
        }
    }

    // std::cout << "Psi: \n" << Psi << std::endl;

    return Psi;
}