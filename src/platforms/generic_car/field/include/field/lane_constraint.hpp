/* 
 * lane_constraint.hpp
 * 
 * Created on: Mar 09, 2018 11:49
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LANE_CONSTRAINT_HPP
#define LANE_CONSTRAINT_HPP

#include <functional>

#include "field/scalar_field.hpp"

namespace librav
{
struct LanePoint
{
    LanePoint(int32_t px, int32_t py) : x(px), y(py){};

    int32_t x;
    int32_t y;
};

typedef std::vector<LanePoint> LanePolylinePoints;

class LaneConstraint : public ScalarField
{
  public:
    LaneConstraint(int64_t size_x = 0, int64_t size_y = 0) : ScalarField(size_x, size_y){};

    void SetLanePolylineKeypoints(const LanePolylinePoints &polyline);

  private:
    LanePolylinePoints lane_keypoints_;
    LanePolylinePoints GenerateIntermediateLanePoints();
    void UpdateConstraint();
};
}

#endif /* LANE_CONSTRAINT_HPP */
