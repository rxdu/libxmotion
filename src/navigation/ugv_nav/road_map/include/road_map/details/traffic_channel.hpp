/* 
 * traffic_channel.hpp
 * 
 * Created on: Oct 18, 2018 08:28
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAFFIC_CHANNEL_HPP
#define TRAFFIC_CHANNEL_HPP

#include <vector>
#include <string>
#include <cstdint>
#include <memory>

#include "geometry/simple_point.hpp"
#include "geometry/polyline.hpp"
#include "geometry/parametric_curve.hpp"

#include "decomp/curvilinear_grid.hpp"

namespace librav
{
class RoadMap;

// struct PathCoordinate
// {
//   PathCoordinate(double _s = 0, double _d = 0) : s(_s), delta(_d) {}
//   double s;
//   double delta;

//   friend std::ostream &operator<<(std::ostream &os, const PathCoordinate &pos)
//   {
//     os << "(s, delta): " << pos.s << " , " << pos.delta;
//     return os;
//   }
// };

class TrafficChannel
{
public:
  TrafficChannel(RoadMap *map, std::string src, std::string dst, std::vector<std::string> lanes);
  ~TrafficChannel() = default;

  std::string source_;
  std::string sink_;
  std::vector<std::string> lanes_;

  Polyline center_line_;
  ParametricCurve center_curve_;
  std::shared_ptr<CurvilinearGrid> grid_;

public:
  void DiscretizeChannel(double step_t, double step_n, int32_t side_num);

private:
  RoadMap *road_map_;

  double origin_offset_ = 0.0;
};
} // namespace librav

#endif /* TRAFFIC_CHANNEL_HPP */
