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
class RVPolygonStateValidityChecker
{
  public:
    using SpaceType = RealVectorSpace<2>;
    using StateType = typename SpaceType::StateType;

  public:
    // RVPolygonStateValidityChecker();

  private:
    std::vector<Polygon> collisions_;
};

class RVPolygonPathValidityChecker
{
  public:
    //   RVPolygonPathValidityChecker(RVPolygonStateValidityChecker)
};
} // namespace librav

#endif /* RV_POLYGON_VALIDITY_CHECKER_HPP */
