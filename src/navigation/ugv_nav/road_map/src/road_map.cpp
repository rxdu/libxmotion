/* 
 * road_map.cpp
 * 
 * Created on: Aug 08, 2018 22:33
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "road_map/road_map.hpp"

#include <cmath>
#include <iostream>

// #include "collision/collision.hpp"

using namespace librav;
using namespace LLet;

RoadMap::RoadMap(std::string map_osm)
{
    if (!map_osm.empty())
        LoadMapFile(map_osm);
}

void RoadMap::LoadMapFile(std::string map_file)
{
    if (map_file.empty())
        return;

    // create lanelet map from osm file
    lanelet_map_ = std::make_unique<LaneletMap>(map_file);
    std::cout << "Load map: " << map_file << std::endl;

    // load all lanelets in the map
    BoundingBox world(std::make_tuple(-180, -180, 180, 180));
    lanelets_ = lanelet_map_->query(world);

    // find reference node with name "origin" and use it as the origin of local reference frame
    bool origin_found = lanelet_map_->get_local_reference_frame_origin(&world_origin_);

    if (!origin_found)
        return;

    // continue setup if origin reference point is found
    coordinate_.SetOrigin(world_origin_);

    // extract lane boundary polylines from all lanelets
    std::vector<double> x_coordinates;
    std::vector<double> y_coordinates;
    for (auto &lanelet : lanelets_)
    {
        ll_id_lookup_.insert(std::make_pair(lanelet->attribute("name").as_string(), lanelet->id()));
        ll_name_lookup_.insert(std::make_pair(lanelet->id(), lanelet->attribute("name").as_string()));

        Polyline left_line, right_line;
        std::vector<point_with_id_t> nodes_right(lanelet->nodes(RIGHT));
        std::vector<point_with_id_t> nodes_left(lanelet->nodes(LEFT));

        for (auto node : nodes_left)
        {
            auto cart = coordinate_.ConvertToCartesian(node);
            left_line.AddPoint(cart.x, cart.y);
            x_coordinates.push_back(cart.x);
            y_coordinates.push_back(cart.y);
        }

        for (auto node : nodes_right)
        {
            auto cart = coordinate_.ConvertToCartesian(node);
            right_line.AddPoint(cart.x, cart.y);
            x_coordinates.push_back(cart.x);
            y_coordinates.push_back(cart.y);
        }

        lane_bounds_.insert(std::make_pair(lanelet->id(), std::make_pair(left_line, right_line)));
        lane_bound_polylines_.push_back(left_line);
        lane_bound_polylines_.push_back(right_line);
    }

    // extract center line points
    for (auto &ln : lanelet_map_->get_lane_center_lines())
    {
        Polyline line;
        for (auto pt : ln.second)
        {
            auto cart = coordinate_.ConvertToCartesian(pt);
            line.AddPoint(cart.x, cart.y);
        }
        lane_center_lines_.insert(std::make_pair(GetLaneletIDByCenterLineName(ln.first), line));
        lane_center_polylines_.push_back(line);
    }

    // find coordinate range
    auto xrange = std::minmax_element(x_coordinates.begin(), x_coordinates.end());
    auto yrange = std::minmax_element(y_coordinates.begin(), y_coordinates.end());

    xmin_ = *(xrange.first);
    xmax_ = *(xrange.second);

    ymin_ = *(yrange.first);
    ymax_ = *(yrange.second);

    // additional information
    GenerateLanePolygon();

    // create topo-geometrical graph
    tg_graph_ = std::make_shared<TopoGeoGraph>(this);
    traffic_map_ = std::make_shared<TrafficMap>(this);

    // set loaded flag
    map_loaded_ = true;
}

void RoadMap::PrintInfo() const
{
    std::cout << "Number of lanelets: " << lanelets_.size() << std::endl;

    for (auto &lanelet : lanelets_)
    {
        std::cout << " - lanelet id: " << lanelet->id() << " , name: " << lanelet->attribute("name").as_string() << std::endl;
    }

    std::cout << "Map region coverage x * y: (" << xmin_ << " , " << xmax_ << ") * (" << ymin_ << " , " << ymax_ << ")" << std::endl;
}

void RoadMap::GenerateLanePolygon()
{
    for (auto &bd : lane_bounds_)
    {
        Polygon polygon(bd.second.first, bd.second.second);
        polygon.ConvexDecomposition();
        lane_polygon_.insert(std::make_pair(bd.first, polygon));
        lane_polygons_.push_back(polygon);
    }
}

/// Returns the lanelet id corresponding to the given center line identified by its name
int32_t RoadMap::GetLaneletIDByCenterLineName(std::string cl_name)
{
    // remove "cl_" to get lanelet name
    cl_name.erase(0, 3);
    return ll_id_lookup_[cl_name];
}

std::pair<Polyline, Polyline> RoadMap::GetLaneBoundaryLines(std::string lane_name)
{
    return lane_bounds_[ll_id_lookup_[lane_name]];
}

Polygon RoadMap::GetLanePolygon(std::string lane_name)
{
    return lane_polygon_[ll_id_lookup_[lane_name]];
}

Polyline RoadMap::GetLaneCenterLine(std::string lane_name)
{
    return lane_center_lines_[ll_id_lookup_[lane_name]];
}

std::vector<int32_t> RoadMap::FindShortestRoute(std::string start_name, std::string goal_name)
{
    auto start = lanelet_map_->lanelet_by_id(ll_id_lookup_[start_name]);
    auto dest = lanelet_map_->lanelet_by_id(ll_id_lookup_[goal_name]);
    std::vector<lanelet_ptr_t> path = lanelet_map_->shortest_path(start, dest);

    std::vector<int32_t> ids;
    for (auto wp : path)
        ids.push_back(wp->id());

    return ids;
}

std::vector<std::string> RoadMap::FindShortestRouteName(std::string start_name, std::string goal_name)
{
    auto ids = FindShortestRoute(start_name, goal_name);
    std::vector<std::string> names;
    for (auto wp : ids)
        names.push_back(ll_name_lookup_[wp]);

    return names;
}

std::vector<int32_t> RoadMap::OccupiedLanelet(CartCooridnate pos)
{
    // query from lanelet map
    // point_with_id_t geo_pt = coordinate_.CreateLaneletPoint(pos);
    // BoundingBox world(geo_pt);

    // auto lanelets = lanelet_map_->query(world);
    // std::cout << "Number of lanelets found at (" << pos.x << " , " << pos.y << ") : " << lanelets.size() << std::endl;
    // // return lanelets;

    // std::vector<int32_t> ids;
    // for (auto ll : lanelets)
    //     ids.push_back(ll->id());

    std::vector<int32_t> ids;
    for (auto &ll : ll_name_lookup_)
    {
        Polygon polygon = GetLanePolygon(ll.second);
        if (polygon.CheckInside(Polygon::Point(pos.x, pos.y)))
        {
            ids.push_back(ll.first);
            // std::cout << "name: " << ll.second << std::endl;
        }
    }

    return ids;
}

bool RoadMap::CheckLaneletCollision(std::string ll1, std::string ll2)
{
    Polygon p1 = GetLanePolygon(ll1);
    Polygon p2 = GetLanePolygon(ll2);
    return p1.Intersect(p2);
}

void RoadMap::CheckLaneletCollision()
{
    for (auto &ll1 : ll_name_lookup_)
    {
        for (auto &ll2 : ll_name_lookup_)
        {
            Polygon p1 = GetLanePolygon(ll1.second);
            Polygon p2 = GetLanePolygon(ll2.second);
            std::cout << "collision " << ll1.second << " - " << ll2.second << " : " << p1.Intersect(p2) << std::endl;
        }
    }
}