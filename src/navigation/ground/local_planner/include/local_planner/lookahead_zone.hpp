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
class LookaheadZone : public CurvilinearGridBase<double, ReferenceTrajectory>
{
  public:
    LookaheadZone() = default;
    LookaheadZone(ReferenceTrajectory curve, double s_step = 2.0);

    ReferenceTrajectory trajectory_;

    // call this function to setup the lookahead zone if it's not setup properly during construction
    void SetupLookaheadZone(ReferenceTrajectory curve, double s_step = 2.0);

  private:
    static constexpr double DStep = 0.7;
    static constexpr double DNum = 3.0;
    static constexpr double SOffset = 0.0;
};
} // namespace librav

#endif /* LOOKAHEAD_ZONE_HPP */
