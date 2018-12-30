/* 
 * planner_base.hpp
 * 
 * Created on: Dec 29, 2018 11:24
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef PLANNER_BASE_HPP
#define PLANNER_BASE_HPP

namespace librav
{
template <typename Space>
class PlannerBase
{
    // , typename Steer, typename ValidityChecker
  public:
    using StateType = typename Space::StateType;
};
} // namespace librav

#endif /* PLANNER_BASE_HPP */
