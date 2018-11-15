/* 
 * lookahead_zone.cpp
 * 
 * Created on: Nov 14, 2018 07:35
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "local_planner/lookahead_zone.hpp"

using namespace librav;

LookaheadZone::LookaheadZone(ReferenceTrajectory curve, double s_step, double d_step, int32_t d_num, double s_offset) : CurvilinearGridBase<double, ReferenceTrajectory>(curve, s_step, d_step, d_num, s_offset)
{
}
