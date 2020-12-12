/* 
 * polyline.cpp
 * 
 * Created on: Aug 09, 2018 06:45
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "geometry/polyline.hpp"

using namespace robotnav;

Polyline::Polyline(std::vector<Point_2> pts)
{
    points_ = pts;
    for (auto &pt : pts)
    {
        double x = CGAL::to_double(pt.x());
        double y = CGAL::to_double(pt.y());
        UpdateXYMinMax(x, y);
    }
}

Polyline::Polyline(std::vector<SimplePoint> pts)
{
    for (auto &pt : pts)
    {
        points_.push_back(Point_2(pt.x, pt.y));
        UpdateXYMinMax(pt.x, pt.y);
    }
}

void Polyline::AddPoint(double x, double y)
{
    UpdateXYMinMax(x, y);
    points_.push_back(Point(x, y));
}

void Polyline::AddPoint(Point pt)
{
    UpdateXYMinMax(CGAL::to_double(pt.x()), CGAL::to_double(pt.y()));
    points_.push_back(pt);
}

bool Polyline::Intersect(const Polyline &other) const
{
    for (int32_t i = 0; i < GetPointNumer() - 1; ++i)
    {
        Segment seg1(points_[i], points_[i + 1]);
        for (int32_t j = 0; j < other.GetPointNumer() - 1; ++j)
        {
            Segment seg2(other.GetPoints()[j], other.GetPoints()[j + 1]);
            if (CGAL::do_intersect(seg1, seg2))
                return true;
        }
    }
    return false;
}

SimplePoint Polyline::GetPoint(std::size_t i) const
{
    assert(i < points_.size());
    return SimplePoint(CGAL::to_double(points_[i].x()), CGAL::to_double(points_[i].y()));
}

std::vector<SimplePoint> Polyline::GetSimplePoints() const
{
    std::vector<SimplePoint> pts;
    for (std::size_t i = 0; i < points_.size(); ++i)
        pts.push_back(SimplePoint(CGAL::to_double(points_[i].x()), CGAL::to_double(points_[i].y())));
    return pts;
}

// Example:
// Polyline1 = {pt1, pt2, pt3}, Polyline2 = {pt3, pt4, pt5}
// Polyline1.SeriesConcatenate(Polyline2) = {pt1, pt2, pt3, pt4, pt5}
Polyline Polyline::SeriesConcatenate(const Polyline &other)
{
    assert(other.GetPointNumer() > 0);

    std::vector<Point> points;
    points.insert(points.end(), points_.begin(), points_.end());

    auto other_pts = other.GetPoints();
    if (points_.empty())
        points.insert(points.end(), other_pts.begin(), other_pts.end());
    else
        points.insert(points.end(), other_pts.begin() + 1, other_pts.end());

    return Polyline(points);
}

Polyline Polyline::operator+(const Polyline &other)
{
    std::vector<Point> points;
    points.insert(points.end(), points_.begin(), points_.end());

    auto other_pts = other.GetPoints();
    points.insert(points.end(), other_pts.begin(), other_pts.end());

    return Polyline(points);
}

void Polyline::PrintInfo() const
{
    std::cout << "Polyline with " << points_.size() << " points" << std::endl;
    for (auto &pt : points_)
        std::cout << "- (" << pt << ")" << std::endl;
}

void Polyline::UpdateXYMinMax(double x, double y)
{
    if (x < xmin_)
        xmin_ = x;
    if (x > xmax_)
        xmax_ = x;
    if (y < ymin_)
        ymin_ = y;
    if (y > ymax_)
        ymax_ = y;
}