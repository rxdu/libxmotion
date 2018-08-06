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
    boundary_points_ = points;

    for (int i = 0; i < points.size() - 1; ++i)
        boundaries_.emplace_back(points[i], points[i + 1]);
}

bool Polygon::InsideConvexPolygon(const PolyLinePoint &pt) 
{
    for (const auto &bound : boundaries_)
    {
        if (!bound.IsOnTheRightSide(pt))
            return false;
    }
    return true;
}

// Source:
//  [1] http://www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
/*
 *  This solution forwarded by Philippe Reverdy is to compute the sum of the angles 
 *  made between the test point and each pair of points making up the polygon. If this 
 *  sum is 2pi then the point is an interior point, if 0 then the point is an exterior 
 *  point. This also works for polygons with holes given the polygon is defined with a 
 *  path made up of coincident edges into and out of the hole as is common practice in 
 *  many CAD packages.
 */
bool Polygon::InsidePolygon(const PolyLinePoint &p)
{
    double angle = 0.0;
    PolyLinePoint p1, p2;

    int n = boundary_points_.size();
    for (int i = 0; i < n; ++i)
    {
        p1.x = boundary_points_[i].x - p.x;
        p1.y = boundary_points_[i].y - p.y;
        p2.x = boundary_points_[(i + 1) % n].x - p.x;
        p2.y = boundary_points_[(i + 1) % n].y - p.y;
        angle += Angle2D(p1.x, p1.y, p2.x, p2.y);
    }

    if (std::abs(angle) < M_PI)
        return false;
    else
        return true;
}

/*
   Return the angle between two vectors on a plane
   The angle is from vector 1 to vector 2, positive anticlockwise
   The result is between -pi -> pi
*/
double Polygon::Angle2D(double x1, double y1, double x2, double y2)
{
    double dtheta, theta1, theta2;

    theta1 = std::atan2(y1, x1);
    theta2 = std::atan2(y2, x2);
    dtheta = theta2 - theta1;
    while (dtheta > M_PI)
        dtheta -= 2 * M_PI;
    while (dtheta < -M_PI)
        dtheta += 2 * M_PI;

    return dtheta;
}