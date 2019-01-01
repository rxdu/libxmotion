/* 
 * validity_checker_base.hpp
 * 
 * Created on: Jan 01, 2019 05:34
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef VALIDITY_CHECKER_BASE_HPP
#define VALIDITY_CHECKER_BASE_HPP

namespace librav
{
template <typename Space>
struct ValidityCheckerBase
{
    using StateType = typename Space::StateType;
    using PathType = std::vector<StateType *>;

    /****************** To Be Implemented ******************/
    // common interface for validity checker
    virtual bool CheckStateValidity(StateType *state) = 0;
    virtual bool CheckPathValidity(StateType *sstate, StateType *dstate) = 0;
    /*******************************************************/
};
} // namespace librav

#endif /* VALIDITY_CHECKER_BASE_HPP */
