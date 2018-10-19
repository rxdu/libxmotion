/* 
 * path_curve.cpp
 * 
 * Created on: Oct 19, 2018 11:51
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "road_map/path_curve.hpp"

#include <vector>

using namespace librav;

PathCurve::PathCurve(Polyline center_polyline) : polyline_(center_polyline)
{
    std::vector<double> distances;
    distances.push_back(0.0);
    for (int32_t i = 0; i < polyline_.GetPointNumer() - 1; ++i)
    {
        double dist = std::hypot(polyline_.GetPoint(i).x - polyline_.GetPoint(i + 1).x,
                                 polyline_.GetPoint(i).y - polyline_.GetPoint(i + 1).y);
        distances.push_back(dist);
    }
}