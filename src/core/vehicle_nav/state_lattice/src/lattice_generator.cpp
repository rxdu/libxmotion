/* 
 * lattice_generator.cpp
 * 
 * Created on: Aug 07, 2018 00:27
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "state_lattice/lattice_generator.hpp"
#include "logging/logger.hpp"

#include <iostream>

using namespace librav;

void LatticeGenerator::SetInitialState(double x, double y, double v, double theta)
{
    init_x_ = x;
    init_y_ = y;
    init_v_ = v;
    init_theta_ = theta;
}

void LatticeGenerator::GenerateControlSet()
{
    // std::vector<double> acc_set = {-9, -6, -3, 0, 1, 2, 3};
    // std::vector<double> acc_set = {-9, -7, -5, -3, -1, 0, 1, 3};
    std::vector<double> acc_set = {0};
    std::vector<double> ster_set; // = {-0.5, -0.4, -0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3, 0.4, 0.5};

    // 1. this set works
    for (int i = -20; i <= 20; ++i)
    {
        ster_set.push_back(i * 0.0245);
    }

    // 2. this set is slow
    // ster_set.push_back(0);
    // for (int i = 1; i <= 15; ++i)
    // {
    //     ster_set.push_back(i * 0.01);
    //     ster_set.push_back(-i * 0.01);
    // }
    // for (int i = 16; i <= 25; ++i)
    // {
    //     ster_set.push_back(0.15 + 0.015 * (i - 15));
    //     ster_set.push_back(-(0.15 + 0.015 * (i - 15)));
    // }
    // for (int i = 26; i <= 32; ++i)
    // {
    //     ster_set.push_back(0.3 + 0.025 * (i - 25));
    //     ster_set.push_back(-(0.3 + 0.025 * (i - 25)));
    // }

    // 3. this set works and slower
    // for (int i = -49; i <= 49; ++i)
    //     ster_set.push_back(i * 0.01);

    ctrl_set_.reserve(acc_set.size() * ster_set.size());
    for (auto acc : acc_set)
        for (auto ster : ster_set)
            ctrl_set_.push_back(BicycleKinematics::control_t(acc, ster));
}

asc::state_t LatticeGenerator::Propagate(asc::state_t init_state, BicycleKinematics::control_t u, double t0, double tf, double dt, int32_t idx)
{
    double t = t0;
    asc::state_t x = init_state;

    while (t < tf)
    {
        integrator_(BicycleKinematics(u), x, t, dt);
        GlobalCsvLogger::GetLogger("mp", "/home/rdu").LogData(idx, init_state[0], init_state[1], init_state[2], init_state[3], x[0], x[1], x[2], x[3]);
    }

    return x;
}

void LatticeGenerator::RunSim(double t0, double tf, double dt)
{
    for (int32_t i = 0; i < ctrl_set_.size(); ++i)
    {
        GlobalCsvLogger::GetLogger("mp", "/home/rdu").LogData(i, init_x_, init_y_, init_v_, init_theta_, init_x_, init_y_, init_v_, init_theta_);
        asc::state_t state = Propagate({init_x_, init_y_, init_v_, init_theta_}, {ctrl_set_[i].accel, ctrl_set_[i].steering}, t0, tf, dt, i);
    }
    std::cout << "Number of generated primitives: " << ctrl_set_.size() << std::endl;
}