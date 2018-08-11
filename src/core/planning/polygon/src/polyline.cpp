/* 
 * polyline.cpp
 * 
 * Created on: Aug 09, 2018 06:45
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "polygon/polyline.hpp"

using namespace librav;

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

void Polyline::AddPoint(double x, double y)
{
    UpdateXYMinMax(x, y);
    points_.push_back(Point(x, y));
}

SimplePoint Polyline::GetPoint(std::size_t i) const
{
    assert(i < points_.size());
    return SimplePoint(CGAL::to_double(points_[i].x()), CGAL::to_double(points_[i].y()));
}

void Polyline::PrintInfo()
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