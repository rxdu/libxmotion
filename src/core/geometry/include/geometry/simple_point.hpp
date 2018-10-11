/* 
 * simple_point.hpp
 * 
 * Created on: Aug 10, 2018 10:56
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SIMPLE_POINT_HPP
#define SIMPLE_POINT_HPP

namespace librav
{
/// Convenient substitution to CGAL Point type
struct SimplePoint
{
    SimplePoint(double _x = 0, double _y = 0) : x(_x), y(_y) {}
    double x;
    double y;
};
} // namespace librav

#endif /* SIMPLE_POINT_HPP */
