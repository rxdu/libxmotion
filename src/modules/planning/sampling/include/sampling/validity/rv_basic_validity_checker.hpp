/* 
 * rv_basic_validity_checker.hpp
 * 
 * Created on: Dec 31, 2018 11:42
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RV_BASIC_VALIDITY_CHECKER_HPP
#define RV_BASIC_VALIDITY_CHECKER_HPP

#include <cstdint>

namespace ivnav
{
template <int32_t N>
class RVBasicStateValidityChecker
{
  public:
    using SpaceType = RealVectorSpace<N>;
    using StateType = typename SpaceType::StateType;

  public:
    RVBasicStateValidityChecker();
};
} // namespace ivnav

#endif /* RV_BASIC_VALIDITY_CHECKER_HPP */
