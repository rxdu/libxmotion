/* 
 * lattice_trajectory.hpp
 * 
 * Created on: Nov 14, 2018 03:27
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LATTICE_TRAJECTORY_HPP
#define LATTICE_TRAJECTORY_HPP

#include <vector>

#include "state_lattice/state_lattice.hpp"

namespace librav
{
class LatticeTrajectory
{
  public:
    LatticeTrajectory(std::vector<StateLattice> path);

    void GenerateConstantSpeedProfile(double speed);

    std::vector<StateLattice> path_;
    double total_length_;
};
} // namespace librav

#endif /* LATTICE_TRAJECTORY_HPP */
