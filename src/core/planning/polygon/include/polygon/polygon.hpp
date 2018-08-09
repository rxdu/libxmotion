/* 
 * polygon.hpp
 * 
 * Created on: Aug 09, 2018 04:15
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef POLYGON_HPP
#define POLYGON_HPP

#include <iostream>
#include <list>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Partition_is_valid_traits_2.h>
#include <CGAL/polygon_function_objects.h>
#include <CGAL/Polygon_2.h>

#include "polygon/polyline.hpp"

namespace librav
{
/// A light wrapper class for CGAL Polygon_2
class Polygon
{
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef CGAL::Partition_traits_2<K> Traits;
    typedef CGAL::Is_convex_2<Traits> Is_convex_2;
    typedef Traits::Polygon_2 Polygon_2;
    typedef Traits::Point_2 Point_2;
    typedef std::list<Polygon_2> Polygon_list;
    typedef CGAL::Partition_is_valid_traits_2<Traits, Is_convex_2> Validity_traits;

  public:
    Polygon() = default;
    Polygon(std::vector<Point_2> pts);
    Polygon(Polyline left_bound, Polyline right_bound);

    using Point = Point_2;
    std::vector<Polygon> convex_partitions_;

    void AddPoint(double x, double y);
    void AddPoint(Point pt);
    void ConvexDecomposition();

    int32_t CheckInside(Point pt);
    inline bool IsSimple() { return data_.is_simple(); }
    inline bool IsConvex() { return data_.is_convex(); }

    // formated output
    void PrintInfo();

  private:
    Polygon_2 data_;
    std::list<Polygon_2> partitions_;
    Traits partition_traits_;
    Validity_traits validity_traits_;

    Polygon(Polygon_2 internal_data) : data_(internal_data) {}
};
} // namespace librav

#endif /* POLYGON_HPP */
