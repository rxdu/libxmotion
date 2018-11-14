/* 
 * lookahead_zone.hpp
 * 
 * Created on: Nov 14, 2018 07:19
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LOOKAHEAD_ZONE_HPP
#define LOOKAHEAD_ZONE_HPP

#include "local_planner/reference_trajectory.hpp"

namespace librav
{
class LookaheadZone
{
  public:
    LookaheadZone(ReferenceTrajectory traj);

  private:
    ReferenceTrajectory trajectory_;
    
};
} // namespace librav

#endif /* LOOKAHEAD_ZONE_HPP */
