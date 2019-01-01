/* 
 * planner_base.hpp
 * 
 * Created on: Dec 29, 2018 11:24
 * Description: a planner maintains a tree built within given space
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef PLANNER_BASE_HPP
#define PLANNER_BASE_HPP

#include <functional>
#include <type_traits>
#include <utility>

#include "sampling/details/base/tree_adapter.hpp"

namespace librav
{
template <typename Space, typename Tree>
class PlannerBase
{
  public:
    using StateType = typename Space::StateType;
    using PathType = typename Tree::PathType;

    using SteerFunc = std::function<std::pair<StateType *, double>(StateType *, StateType *)>;
    using StateValidityCheckFunc = std::function<bool(StateType *state)>;
    using PathValidityCheckFunc = std::function<bool(StateType *sstate, StateType *dstate)>;

    /****************** type sanity check ******************/
    // check if the tree interface is satisfied
    static_assert(std::is_base_of<TreeAdapter<Space>, Tree>::value,
                  "Tree should inherit from TreeAdapter to enforce interface compatibility");
    /*******************************************************/

  public:
    PlannerBase(Space *s) : space_(s), tree_(space_) {}

    // helper functions
    void SetSteerFunction(SteerFunc func) { Steer = func; }
    void SetStateValidityChecker(StateValidityCheckFunc func) { CheckStateValidity = func; }
    void SetPathValidityChecker(PathValidityCheckFunc func) { CheckPathValidity = func; }

    /****************** To Be Implemented ******************/
    // common interface for planner
    virtual PathType Search(StateType *start, StateType *goal, int32_t iter = -1) = 0;
    /*******************************************************/

  protected:
    Space *space_;
    Tree tree_;

    SteerFunc Steer = nullptr;
    StateValidityCheckFunc CheckStateValidity =
        std::bind(&PlannerBase<Space, Tree>::DefaultStateValidityCheck, this, std::placeholders::_1);
    PathValidityCheckFunc CheckPathValidity =
        std::bind(&PlannerBase<Space, Tree>::DefaultPathValidityCheck, this, std::placeholders::_1, std::placeholders::_2);

    bool CheckGoal(StateType *state, StateType *goal, double dist)
    {
        if (space_->EvaluateDistance(state, goal) <= dist)
            return true;
        else
            return false;
    }

    // default validity check function - always valid
    bool DefaultStateValidityCheck(StateType *state) { return true; }
    bool DefaultPathValidityCheck(StateType *sstate, StateType *dstate) { return true; }
};
} // namespace librav

#endif /* PLANNER_BASE_HPP */
