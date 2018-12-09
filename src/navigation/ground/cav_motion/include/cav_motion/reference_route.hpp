/* 
 * reference_route.hpp
 * 
 * Created on: Dec 09, 2018 07:47
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef REFERENCE_ROUTE_HPP
#define REFERENCE_ROUTE_HPP

#include <string>
#include <memory>

#include "common/librav_types.hpp"
#include "traffic_map/traffic_channel.hpp"

namespace librav
{
class ReferenceRoute : public TrafficChannel
{
  public:
    ReferenceRoute() = default;
    ReferenceRoute(std::shared_ptr<RoadMap> map, Position2d ps, Position2d pg,
                   std::string src, std::string dst, std::vector<std::string> lanes);

    Position2d position_s;
    Position2d position_g;

    std::string lanelet_s;
    std::string lanelet_g;

    CurvilinearGrid::GridPoint init_route_point_;

    void PrintInfo();
};
} // namespace librav

#endif /* REFERENCE_ROUTE_HPP */
