/* 
 * road_coordinate.hpp
 * 
 * Created on: Apr 03, 2018 17:18
 * Description: it's assumed that the map is defined in north america and
 *          the origin is defined at the south-west corner on the map
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef ROAD_COORDINATE_HPP
#define ROAD_COORDINATE_HPP

namespace librav
{
class RoadCoordinate
{
public:
  RoadCoordinate() = default;

  void SetRange(double x_min, double x_max, double y_min, double y_max);

private:
  double x_min_ = 0.0;
  double x_max_ = 0.0;
  double y_min_ = 0.0;
  double y_max_ = 0.0;
};
}

#endif /* ROAD_COORDINATE_HPP */
