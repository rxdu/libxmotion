/* 
 * road_map.cpp
 * 
 * Created on: Apr 03, 2018 14:18
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "road_network/road_map.hpp"

#include <cmath>
#include <iostream>

using namespace librav;
using namespace LLet;

RoadMap::RoadMap(std::string map_osm)
{
    if (!map_osm.empty())
        LoadMapFile(map_osm);
}

void RoadMap::LoadMapFile(std::string map_file)
{
    // create lanelet map from osm file
    lanelet_map_ = std::make_unique<LaneletMap>(map_file);
    std::cout << "Map loaded: " << map_file << std::endl;

    // load all lanelets in the map
    BoundingBox world(std::make_tuple(-180, -180, 180, 180));
    lanelets_ = lanelet_map_->query(world);
    std::cout << "Number of lanelets found: " << lanelets_.size() << std::endl;

    // find lanelet 0 and use the second point of the left bound as origin
    world_origin_ = lanelet_map_->lanelet_by_id(ref_lanelet_id_)->nodes(LEFT)[1];

    // extract lane bounds from all lanelets
    std::vector<double> x_coordinates;
    std::vector<double> y_coordinates;
    for (auto &lanelet : lanelets_)
    {
        // lanelet with id "ref_lanelet_id_" is reserved for coordinate reference
        if (lanelet->id() == ref_lanelet_id_)
            continue;

        PolyLine left_line, right_line;
        std::vector<point_with_id_t> nodes_right(lanelet->nodes(RIGHT));
        std::vector<point_with_id_t> nodes_left(lanelet->nodes(LEFT));

        for (auto node : nodes_left)
        {
            auto left_vec = LLet::vec(world_origin_, node);
            left_line.AddPoint(left_vec.first, left_vec.second);
            x_coordinates.push_back(left_vec.first);
            y_coordinates.push_back(left_vec.second);
        }

        for (auto node : nodes_right)
        {
            auto right_vec = LLet::vec(world_origin_, node);
            right_line.AddPoint(right_vec.first, right_vec.second);
            x_coordinates.push_back(right_vec.first);
            y_coordinates.push_back(right_vec.second);
        }

        lane_bounds_.insert(std::make_pair(lanelet->id(), std::make_pair(left_line, right_line)));
    }

    // find coordinate range
    auto xrange = std::minmax_element(x_coordinates.begin(), x_coordinates.end());
    auto yrange = std::minmax_element(y_coordinates.begin(), y_coordinates.end());
    x_min_ = *(xrange.first);
    x_max_ = *(xrange.second);
    y_min_ = *(yrange.first);
    y_max_ = *(yrange.second);
    std::cout << "coordinate range x " << x_min_ << " , " << x_max_ << std::endl;
    std::cout << "coordinate range y " << y_min_ << " , " << y_max_ << std::endl;
}

void RoadMap::CreateDenseGrid(int32_t pixel_per_meter)
{
    for (auto &bound : lane_bounds_)
    {
        std::cout << "lanelet id: " << bound.first << std::endl;
        bound.second.first.PrintPoints();
        std::cout << " --- " << std::endl;
        bound.second.second.PrintPoints();
        std::cout << " ------ " << std::endl;
    }
}