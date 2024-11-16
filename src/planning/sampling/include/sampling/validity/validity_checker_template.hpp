/* 
 * validity_checker_template.hpp
 * 
 * Created on: Jan 01, 2019 10:20
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#ifndef VALIDITY_CHECKER_TEMPLATE_HPP
#define VALIDITY_CHECKER_TEMPLATE_HPP

#include <vector>
#include <cstdint>

#include "sampling/space/realvector_space.hpp"

namespace xmotion
{
class ValidityCheckerTemplate
{
  public:
    // 2-D RealVectorSpace is used as an example
    //  replace it with other space types
    using SpaceType = RealVectorSpace<2>;
    using StateType = typename SpaceType::StateType;

  public:
    // operator for state validity checking
    bool operator()(std::shared_ptr<StateType>state)
    {
        return true;
    }

    // operator for path validity checking
    bool operator()(std::shared_ptr<StateType>sstate, std::shared_ptr<StateType>dstate)
    {
        return true;
    }

    // add other member variables/functions
};
} // namespace xmotion

#endif /* VALIDITY_CHECKER_TEMPLATE_HPP */
