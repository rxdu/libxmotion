/* 
 * map_loader.hpp
 * 
 * Created on: Nov 02, 2018 05:07
 * Description: a convenicence class to load road map and 
 *              create associated traffic map
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MAP_LOADER_HPP
#define MAP_LOADER_HPP

#include <string>
#include <memory>
#include <iostream>

#include "road_map/road_map.hpp"
#include "traffic_map/traffic_map.hpp"
#include "stopwatch/stopwatch.h"

namespace ivnav
{
struct MapLoader
{
    MapLoader(std::string map_file)
    {
        stopwatch::StopWatch timer;

        road_map = std::make_shared<RoadMap>(map_file);

        if (!road_map->MapReady())
        {
            std::cerr << "ERROR: map didn't load correctly" << std::endl;
            return;
        }

        road_map->PrintInfo();

        traffic_map = std::make_shared<TrafficMap>(road_map);

        std::cout << "INFO: map loaded in " << timer.toc() << " seconds" << std::endl;
        map_ready = true;
    }

    bool map_ready = false;

    std::shared_ptr<RoadMap> road_map;
    std::shared_ptr<TrafficMap> traffic_map;
};
} // namespace ivnav

#endif /* MAP_LOADER_HPP */
