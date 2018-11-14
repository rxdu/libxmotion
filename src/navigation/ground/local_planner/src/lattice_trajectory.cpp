/* 
 * lattice_trajectory.cpp
 * 
 * Created on: Nov 14, 2018 03:31
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "local_planner/lattice_trajectory.hpp"

using namespace librav;

LatticeTrajectory::LatticeTrajectory(std::vector<StateLattice> path) : path_(path)
{
}

void LatticeTrajectory::GenerateConstantSpeedProfile(double speed)
{
    total_length_ = 0;
    for (auto &lattice :path_)
        total_length_ += lattice.GetLength();
    std::cout << "total length: " << total_length_ << std::endl;
}