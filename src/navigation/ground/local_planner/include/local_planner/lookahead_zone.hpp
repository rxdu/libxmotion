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

#include "decomp/curvilinear_grid.hpp"
#include "local_planner/reference_trajectory.hpp"

namespace librav
{
class LookaheadZone: public CurvilinearGridBase<double, ReferenceTrajectory>
{
  public:
    LookaheadZone(ReferenceTrajectory curve, double s_step = 2.0, double d_step = 0.7, int32_t d_num = 3, double s_offset = 0);

  private:
    ReferenceTrajectory trajectory_;
    
};
} // namespace librav

#endif /* LOOKAHEAD_ZONE_HPP */
