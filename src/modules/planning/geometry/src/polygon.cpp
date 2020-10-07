/* 
 * polygon.cpp
 * 
 * Created on: Aug 09, 2018 04:16
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "geometry/polygon.hpp"

#include <iterator>
#include <cassert>
#include <cmath>

#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/partition_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Point_2.h>

using namespace autodrive;

Polygon::Polygon(std::initializer_list<SimplePoint> pts)
{
    for (auto &pt : pts)
        AddPoint(pt.x, pt.y);
}

Polygon::Polygon(std::vector<Point_2> pts)
{
    for (auto &pt : pts)
        data_.push_back(pt);
}

Polygon::Polygon(Polyline left_bound, Polyline right_bound)
{
    // add points from the right bound first
    for (auto &pt : right_bound.GetPoints())
        data_.push_back(pt);
    // add points from the left bound in the reversed order
    auto pts = left_bound.GetPoints();
    for (auto it = pts.rbegin(); it < pts.rend(); ++it)
        data_.push_back(*it);
}

/// Polygon tranformation: rotate first, then translate
Polygon Polygon::TransformRT(double dx, double dy, double dtheta)
{
    Polygon new_polygon;
    for (auto it = data_.vertices_begin(); it != data_.vertices_end(); ++it)
    {
        double x = CGAL::to_double((*it).x()) * std::cos(dtheta) - CGAL::to_double((*it).y()) * std::sin(dtheta) + dx;
        double y = CGAL::to_double((*it).x()) * std::sin(dtheta) + CGAL::to_double((*it).y()) * std::cos(dtheta) + dy;
        new_polygon.AddPoint(x, y);
    }
    return new_polygon;
}

/// Polygon tranformation: translate first, then rotate
Polygon Polygon::TransformTR(double dx, double dy, double dtheta)
{
    Polygon new_polygon;
    for (auto it = data_.vertices_begin(); it != data_.vertices_end(); ++it)
    {
        double x = CGAL::to_double((*it).x() + dx) * std::cos(dtheta) - CGAL::to_double((*it).y() + dy) * std::sin(dtheta);
        double y = CGAL::to_double((*it).x() + dx) * std::sin(dtheta) + CGAL::to_double((*it).y() + dy) * std::cos(dtheta);
        new_polygon.AddPoint(x, y);
    }
    return new_polygon;
}

void Polygon::PrintInfo() const
{
    std::cout << "Polygon with " << data_.size() << " points" << std::endl;
    for (auto it = data_.vertices_begin(); it != data_.vertices_end(); ++it)
        std::cout << "- (" << *it << ")" << std::endl;
}

void Polygon::AddPoint(double x, double y)
{
    data_.push_back(Point(x, y));
}

void Polygon::AddPoint(Point pt)
{
    data_.push_back(pt);
}

bool Polygon::CheckInside(Point pt) const
{
    if (CGAL::bounded_side_2(data_.vertices_begin(), data_.vertices_end(), pt, K()) == CGAL::ON_BOUNDED_SIDE)
        return true;
    return false;
}

bool Polygon::CheckInside(double x, double y) const
{
    if (CGAL::bounded_side_2(data_.vertices_begin(), data_.vertices_end(), Point(x, y), K()) == CGAL::ON_BOUNDED_SIDE)
        return true;
    return false;
}

int32_t Polygon::CheckRelativePosition(Point pt) const
{
    switch (CGAL::bounded_side_2(data_.vertices_begin(), data_.vertices_end(), pt, K()))
    {
    case CGAL::ON_BOUNDED_SIDE:
        return 1;
    case CGAL::ON_BOUNDARY:
        return 0;
    case CGAL::ON_UNBOUNDED_SIDE:
        return -1;
    }
}

bool Polygon::Intersect(const Polygon &other) const
{
    return CGAL::do_intersect(data_, other.data_);
}

bool Polygon::Contain(const Polygon &other) const
{
    for (auto it = other.point_begin(); it != other.point_end(); ++it)
    {
        if (!CheckInside(*it))
            return false;
    }
    return true;
}

SimplePoint Polygon::GetPoint(std::size_t i) const
{
    assert(i < data_.size());
    return SimplePoint(CGAL::to_double(data_[i].x()), CGAL::to_double(data_[i].y()));
}

void Polygon::ConvexDecomposition()
{
    convex_partitions_.clear();

    if (data_.is_convex())
        convex_partitions_.push_back(Polygon(data_));

    CGAL::optimal_convex_partition_2(data_.vertices_begin(),
                                     data_.vertices_end(),
                                     std::back_inserter(partitions_),
                                     partition_traits_);

    assert(CGAL::partition_is_valid_2(data_.vertices_begin(),
                                      data_.vertices_end(),
                                      partitions_.begin(),
                                      partitions_.end(),
                                      validity_traits_));

    for (auto it = partitions_.begin(); it != partitions_.end(); ++it)
        convex_partitions_.push_back(Polygon(*it));
}