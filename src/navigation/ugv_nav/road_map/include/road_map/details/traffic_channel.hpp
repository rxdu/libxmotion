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

#include "geometry/simple_point.hpp"
#include "geometry/polyline.hpp"
#include "geometry/parametric_curve.hpp"

namespace librav
{
class RoadMap;

struct PathCoordinate
{
  PathCoordinate(double _s = 0, double _d = 0) : s(_s), delta(_d) {}
  double s;
  double delta;

  friend std::ostream &operator<<(std::ostream &os, const PathCoordinate &pos)
  {
    os << "(s, delta): " << pos.s << " , " << pos.delta;
    return os;
  }
};

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

  void DiscretizeChannel(double step_t, double step_n);

  // local path coordinate
  // - s: starts from 0, but be offset by "origin_offset_"
  // - delta: positive - left, zero - on curve, negative -right 
  double GetOriginOffset() const { return origin_offset_; }
  void SetOriginOffset(double offset) { origin_offset_ = offset; }
  PathCoordinate ConvertToPathCoordinate(SimplePoint pt);
  SimplePoint ConvertToGlobalCoordinate(PathCoordinate pt);

private:
  RoadMap *road_map_;

  double origin_offset_ = 0.0;
};
} // namespace librav

#endif /* TRAFFIC_CHANNEL_HPP */
