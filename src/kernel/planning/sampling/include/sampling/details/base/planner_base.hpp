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

#include "sampling/details/base/tree_adapter.hpp"
#include "sampling/details/base/validity_checker_base.hpp"

namespace librav
{
template <typename Space, typename Tree>
class PlannerBase
{
    struct DefaultValidityChecker;

  public:
    using StateType = typename Space::StateType;
    using PathType = typename Tree::PathType;

    using SteerFunc = std::function<StateType *(StateType *, StateType *)>;
    
    /****************** type sanity check ******************/
    // check if the tree interface is satisfied
    static_assert(std::is_base_of<TreeAdapter<Space>, Tree>::value,
                  "Tree should inherit from TreeAdapter to enforce interface compatibility");
    /*******************************************************/

  public:
    PlannerBase(Space *s) : space_(s), tree_(space_),
                            default_validity_checker_(new DefaultValidityChecker())
    {
        validity_checker_ = default_validity_checker_;
    }

    virtual ~PlannerBase()
    {
        // note: planner will only deallocated the default validity checker
        //      the user is responsible for any external checkers
        delete default_validity_checker_;
    }

    Space *space_;
    Tree tree_;

    SteerFunc Steer = nullptr;
    ValidityCheckerBase<Space> *validity_checker_ = nullptr;

    // helper functions
    void SetSteerFunction(SteerFunc func) { Steer = func; }
    void SetValidityChecker(ValidityCheckerBase<Space> *validity_checker) { validity_checker_ = validity_checker; }

    bool CheckGoal(StateType *state, StateType *goal, double dist)
    {
        if (space_->EvaluateDistance(state, goal) <= dist)
            return true;
        else
            return false;
    }

    /****************** To Be Implemented ******************/
    // common interface for planner
    virtual PathType Search(StateType *start, StateType *goal, int32_t iter = -1) = 0;
    /*******************************************************/

  private:
    DefaultValidityChecker *default_validity_checker_ = nullptr;

    struct DefaultValidityChecker : public ValidityCheckerBase<Space>
    {
        bool CheckStateValidity(StateType *state) final { return true; };
        bool CheckPathValidity(StateType *sstate, StateType *dstate) final { return true; };
    };
};
} // namespace librav

#endif /* PLANNER_BASE_HPP */
