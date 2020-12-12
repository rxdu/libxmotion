/* 
 * simple_point.hpp
 * 
 * Created on: Aug 10, 2018 10:56
 * Description: a convenient point type for the CGAL wrappers
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SIMPLE_POINT_HPP
#define SIMPLE_POINT_HPP

#include <iostream>

namespace robotnav
{
/// Convenient substitution to CGAL Point type
struct SimplePoint
{
    SimplePoint(double _x = 0, double _y = 0) : x(_x), y(_y) {}
    double x;
    double y;

    friend std::ostream &operator<<(std::ostream &os, const SimplePoint &pos)
    {
        os << "(x, y): " << pos.x << " , " << pos.y;
        return os;
    }
};
} // namespace robotnav

#endif /* SIMPLE_POINT_HPP */
