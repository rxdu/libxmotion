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

#include "geometry/polyline.hpp"
#include "road_map/path_curve.hpp"

namespace librav
{
class RoadMap;

class TrafficChannel
{
public:
  TrafficChannel(RoadMap *map, std::string src, std::string dst, std::vector<std::string> lanes);
  ~TrafficChannel() = default;

  std::string source_;
  std::string sink_;
  std::vector<std::string> lanes_;

  Polyline center_line_;
  PathCurve center_curve_;

private:
  RoadMap *road_map_;

  void FitCurve();
};
} // namespace librav

#endif /* TRAFFIC_CHANNEL_HPP */
