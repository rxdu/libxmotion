/* 
 * lane_constraint.cpp
 * 
 * Created on: Mar 09, 2018 12:36
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "threat_field/lane_constraint.hpp"

using namespace librav;

void LaneConstraint::SetLanePolylineKeypoints(const LanePolylinePoints &polyline)
{
    lane_keypoints_ = polyline;

    UpdateConstraint();
}

void LaneConstraint::UpdateConstraint()
{
    LanePolylinePoints points = GenerateIntermediateLanePoints();
    for (const auto &pt : points)
        SetValueAtCoordinate(pt.x, pt.y, 1);
}

LanePolylinePoints LaneConstraint::GenerateIntermediateLanePoints()
{
    LanePolylinePoints points;
    for (int i = 0; i < lane_keypoints_.size() - 1; ++i)
    {
        LanePoint pt1(lane_keypoints_[i].x, lane_keypoints_[i].y);
        LanePoint pt2(lane_keypoints_[i + 1].x, lane_keypoints_[i + 1].y);

        int32_t x_err = pt2.x - pt1.x;
        int32_t y_err = pt2.y - pt1.y;

        if (x_err == 0)
        {
            if (y_err > 0)
            {
                for (int32_t y = pt1.y; y <= pt2.y; ++y)
                    points.emplace_back(pt1.x, y);
            }
            else
            {
                for (int32_t y = pt1.y; y >= pt2.y; --y)
                    points.emplace_back(pt1.x, y);
            }
        }
        else
        {
            double k = y_err / x_err;
            if (x_err > 0)
            {
                for (int32_t x = pt1.x; x <= pt2.x; ++x)
                    points.emplace_back(x, static_cast<int32_t>(pt1.y + (x - pt1.x) * k));
            }
            else
            {
                for (int32_t x = pt1.x; x >= pt2.x; --x)
                {
                    if (k!= 0 && static_cast<int32_t>(pt1.y + (x - pt1.x) * k) - 1 > 0)
                        points.emplace_back(x, static_cast<int32_t>(pt1.y + (x - pt1.x) * k) - 1);
                    points.emplace_back(x, static_cast<int32_t>(pt1.y + (x - pt1.x) * k));
                    if (k!= 0 && static_cast<int32_t>(pt1.y + (x - pt1.x) * k) + 1 < SizeY())
                        points.emplace_back(x, static_cast<int32_t>(pt1.y + (x - pt1.x) * k) + 1);
                }
            }
        }
    }

    return points;
}