/* 
 * markov_occupancy.cpp
 * 
 * Created on: Oct 29, 2018 07:50
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "reachability/markov_occupancy.hpp"

using namespace librav;

MarkovOccupancy::MarkovOccupancy()
{
    SetupCommandModel();
}

void MarkovOccupancy::SetupCommandModel()
{
    CommandModel::State init_state;
    init_state << 0, 0, 0.5, 0.5, 0, 0;

    CommandModel::ControlSet cmds;
    cmds << -1, -0.5, 0, 0.3, 0.6, 1.0;

    CommandModel::PriorityVector priority_vec;
    priority_vec << 0.01, 0.04, 0.25, 0.25, 0.4, 0.05;

    double gamma = 0.2;

    command_.SetupModel(init_state, cmds, priority_vec, gamma);
}