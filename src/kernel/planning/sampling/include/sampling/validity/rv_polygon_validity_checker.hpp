/* 
 * rv_polygon_validity_checker.hpp
 * 
 * Created on: Dec 31, 2018 11:42
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RV_POLYGON_VALIDITY_CHECKER_HPP
#define RV_POLYGON_VALIDITY_CHECKER_HPP

#include <vector>
#include <cstdint>

#include "geometry/polygon.hpp"
#include "sampling/space/realvector_space.hpp"

namespace librav
{
class RVPolygonValidityChecker
{
  public:
    using SpaceType = RealVectorSpace<2>;
    using StateType = typename SpaceType::StateType;

  public:
    bool operator()(StateType *state)
    {
        return true;
    }

    bool operator()(StateType *sstate, StateType *dstate)
    {
        return true;
    }

    void AddCollisionPolygon()
    {
    }

  private:
    std::vector<Polygon> collisions_;
};
} // namespace librav

#endif /* RV_POLYGON_VALIDITY_CHECKER_HPP */
