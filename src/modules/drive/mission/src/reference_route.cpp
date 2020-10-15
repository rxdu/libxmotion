/* 
 * reference_route.cpp
 * 
 * Created on: Dec 09, 2018 07:50
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "mission/reference_route.hpp"

#include <iostream>

namespace ivnav
{
ReferenceRoute::ReferenceRoute(std::shared_ptr<RoadMap> map, Position2d ps, Position2d pg,
                               std::string src, std::string dst, std::vector<std::string> lanes) : TrafficChannel(map, src, dst, lanes),
                                                                                                   position_s(ps), position_g(pg),
                                                                                                   lanelet_s(src), lanelet_g(dst)
{
    init_route_point_ = ConvertToPathCoordinate({ps.x, ps.y});
    std::cout << "init s: " << init_route_point_.s << std::endl;
    DiscretizeChannel(init_route_point_.s, 5, 0.74, 5);
}

void ReferenceRoute::PrintInfo()
{
    std::cout << "(start) -> ";
    for (auto &ll : lanes_)
        std::cout << ll << " -> ";
    std::cout << "(goal)" << std::endl;
}
} // namespace ivnav