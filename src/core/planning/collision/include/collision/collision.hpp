/* 
 * collision.hpp
 * 
 * Created on: Aug 09, 2018 01:39
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef COLLISION_HPP
#define COLLISION_HPP

#include "polygon/polygon.hpp"

namespace librav
{
namespace Collision
{
/// This function assumes convex decomposition has been performed for 
/// both polygon1 and polygon2.
bool Check(Polygon polygon1, Polygon polygon2);
};
} // namespace librav

#endif /* COLLISION_HPP */
