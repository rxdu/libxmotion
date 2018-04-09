/* 
 * road_geometry.cpp
 * 
 * Created on: Apr 05, 2018 13:35
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "road_network/road_geometry.hpp"

#include <iostream>

#include <Eigen/Geometry>

using namespace librav;

PolyLineSegment::PolyLineSegment(PolyLinePoint pt1, PolyLinePoint pt2)
{
    start_point_ = pt1;
    end_point_ = pt2;

    base_vec_ << end_point_.x - start_point_.x, end_point_.y - start_point_.y, 0;
    // std::cout << "base vector: " << base_vec_(0) << " , " << base_vec_(1) << std::endl;
}

bool PolyLineSegment::IsOnTheRightSide(const PolyLinePoint &pt) const
{
    Eigen::Vector3d check_vec(pt.x - start_point_.x, pt.y - start_point_.y, 0);
    // std::cout << "check vector: " << check_vec(0) << " , " << check_vec(1) << " sign: " << check_vec.cross(base_vec_)(2) << std::endl;

    if (check_vec.cross(base_vec_)(2) >= 0)
        return true;
    else
        return false;
}

////////////////////////////////////////////////////////////

void PolyLine::AddPoint(double x, double y)
{
    // points_.insert(std::make_pair(point_num_++, PolyLinePoint(x, y)));
    points_.emplace_back(x, y);
    point_num_++;
}

void PolyLine::PrintPoints() const
{
    for (auto &pt : points_)
        std::cout << pt.x << " , " << pt.y << std::endl;
}

////////////////////////////////////////////////////////////

Polygon::Polygon(const std::vector<PolyLinePoint> &points)
{
    SetBoundary(points);
}

void Polygon::SetBoundary(const std::vector<PolyLinePoint> &points)
{
    // boundary_points_ = points;

    for (int i = 0; i < points.size() - 1; ++i)
        boundaries_.emplace_back(points[i], points[i + 1]);
}

bool Polygon::IsInside(const PolyLinePoint &pt) const
{
    for (const auto &bound : boundaries_)
    {
        if (!bound.IsOnTheRightSide(pt))
            return false;
    }
    return true;
}