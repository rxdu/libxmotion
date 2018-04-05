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

RoadMap::RoadMap(std::string map_osm, int32_t ppm) : pixel_per_meter_(ppm)
{
    if (!map_osm.empty())
        LoadMapFile(map_osm, pixel_per_meter_);
}

bool RoadMap::LoadMapFile(std::string map_file, int32_t ppm)
{
    if (map_file.empty())
        return false;

    // create lanelet map from osm file
    lanelet_map_ = std::make_unique<LaneletMap>(map_file);
    std::cout << "Map loaded: " << map_file << std::endl;

    // load all lanelets in the map
    BoundingBox world(std::make_tuple(-180, -180, 180, 180));
    lanelets_ = lanelet_map_->query(world);
    std::cout << "Number of lanelets found: " << lanelets_.size() << std::endl;

    // find lanelet with name "origin" and use the second point of the left bound as origin
    bool origin_found = false;
    for (auto &lanelet : lanelets_)
    {
        if (lanelet->attribute("name").as_string() == "origin")
        {
            world_origin_ = lanelet->nodes(LEFT)[1];
            ref_lanelet_id_ = lanelet->id();
            origin_found = true;
        }
    }

    if (!origin_found)
        return false;

    // continue setup if origin lanelet found
    coordinate_.SetOrigin(world_origin_);

    // extract lane bounds from all lanelets
    std::vector<double> x_coordinates;
    std::vector<double> y_coordinates;
    for (auto &lanelet : lanelets_)
    {
        // lanelet with id "ref_lanelet_id_" is used for coordinate reference only
        if (lanelet->id() == ref_lanelet_id_)
            continue;

        std::cout << "lanelet info: id " << lanelet->id() << " , name " << lanelet->attribute("name").as_string() << std::endl;
        ll_id_lookup_.insert(std::make_pair(lanelet->attribute("name").as_string(), lanelet->id()));
        ll_name_lookup_.insert(std::make_pair(lanelet->id(), lanelet->attribute("name").as_string()));

        PolyLine left_line, right_line;
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
    }

    // find coordinate range
    auto xrange = std::minmax_element(x_coordinates.begin(), x_coordinates.end());
    auto yrange = std::minmax_element(y_coordinates.begin(), y_coordinates.end());

    x_min_ = *(xrange.first);
    x_max_ = *(xrange.second);

    y_min_ = *(yrange.first);
    y_max_ = *(yrange.second);

    // generate dense grids for planning
    GenerateDenseGrids(ppm);

    return true;
}

void RoadMap::GenerateDenseGrids(int32_t pixel_per_meter)
{
    // add 1 to each dimension to compensate for the rounding
    grid_size_x_ = static_cast<int32_t>(x_max_ * pixel_per_meter) + 1;
    grid_size_y_ = static_cast<int32_t>(y_max_ * pixel_per_meter) + 1;

    coordinate_.SetDenseGridSize(grid_size_x_, grid_size_y_, pixel_per_meter);

    std::cout << "dense grid size: " << grid_size_x_ << " , " << grid_size_y_ << " on region " << x_min_ << " , " << x_max_ << " , " << y_min_ << " , " << y_max_ << std::endl;

    for (auto &bound : lane_bounds_)
    {
        auto sub_grid = std::make_shared<DenseGrid>(grid_size_x_, grid_size_y_);

        // std::cout << "lanelet id: " << bound.first << std::endl;
        std::vector<DenseGridPixel> points_left = GenerateLanePoints(bound.second.first);
        std::vector<DenseGridPixel> points_right = GenerateLanePoints(bound.second.second);
        for (const auto &pt : points_left)
            sub_grid->SetValueAtCoordinate(pt.x, pt.y, 1.0);
        for (const auto &pt : points_right)
            sub_grid->SetValueAtCoordinate(pt.x, pt.y, 1.0);

        lane_grids_.insert(std::make_pair(bound.first, sub_grid));
    }

    // auto bound = lane_bounds_[-39200];
    // std::vector<DenseGridPixel> points_left = GenerateLanePoints(bound.first, resolution);
    // // std::vector<DenseGridPixel> points_right = GenerateLanePoints(bound.second, resolution);
    // for (const auto &pt : points_left)
    //     full_grid->SetValueAtCoordinate(pt.x, pt.y, 1.0);
    // // for (const auto &pt : points_right)
    // //     full_grid->SetValueAtCoordinate(pt.x, pt.y, 1.0);
}

std::shared_ptr<DenseGrid> RoadMap::GetFullGrid()
{
    auto full_grid = std::make_shared<DenseGrid>(grid_size_x_, grid_size_y_);
    for (auto grid : lane_grids_)
    {
        full_grid->AddGrid(*grid.second.get());
    }
    return full_grid;
}

std::shared_ptr<DenseGrid> RoadMap::GetLaneBoundsGrid(std::vector<std::string> lanelets)
{
    auto grid = std::make_shared<DenseGrid>(grid_size_x_, grid_size_y_);
    for (const auto &ll_name : lanelets)
    {
        grid->AddGrid(*lane_grids_[ll_id_lookup_[ll_name]].get());
    }
    return grid;
}

std::shared_ptr<DenseGrid> RoadMap::GetDrivableAreaGrid()
{
    // check whether a position is inside a lanelet
}

std::shared_ptr<DenseGrid> RoadMap::GetLaneBoundsGrid(std::vector<int32_t> lanelets)
{
    auto grid = std::make_shared<DenseGrid>(grid_size_x_, grid_size_y_);
    for (const auto &ll_id : lanelets)
    {
        grid->AddGrid(*lane_grids_[ll_id].get());
    }
    return grid;
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

std::vector<DenseGridPixel> RoadMap::GenerateLanePoints(const PolyLine &line)
{
    std::vector<DenseGridPixel> grid_points;
    std::vector<DenseGridPixel> interpolated_points;

    for (const auto &pt : line.points_)
    {
        // for (int i = 0; i < 3; ++i)
        // {
        // auto pt = line.points_[i];
        auto gpt = coordinate_.ConvertToGridPixel(CartCooridnate(pt.x, pt.y));
        // std::cout << "line point: " << gpt.x << " , " << gpt.y << std::endl;
        grid_points.push_back(gpt);
    }

    for (int i = 0; i < grid_points.size() - 1; ++i)
    // for (int i = 0; i < 2; ++i)
    {
        std::vector<DenseGridPixel> pts = InterpolateGridPixelPoints(grid_points[i], grid_points[i + 1]);

        interpolated_points.insert(interpolated_points.end(), pts.begin(), pts.end());
    }

    return interpolated_points;
}

std::vector<DenseGridPixel> RoadMap::InterpolateGridPixelPoints(DenseGridPixel pt1, DenseGridPixel pt2)
{
    std::vector<DenseGridPixel> points;

    int32_t x_err = pt2.x - pt1.x;
    int32_t y_err = pt2.y - pt1.y;

    // std::cout << "xerr: " << x_err << " , y_err: " << y_err << std::endl;

    DenseGridPixel start_pt, end_pt;

    // if vertical line
    if (x_err == 0)
    {
        // find lower point
        if (y_err > 0)
        {
            start_pt = pt1;
            end_pt = pt2;
        }
        else
        {
            start_pt = pt2;
            end_pt = pt1;
        }

        for (int32_t y = start_pt.y; y <= end_pt.y; ++y)
            points.emplace_back(pt1.x, y);
    }
    // if horizontal line
    else if (y_err == 0)
    {
        // find left point
        if (x_err > 0)
        {
            start_pt = pt1;
            end_pt = pt2;
        }
        else
        {
            start_pt = pt2;
            end_pt = pt1;
        }
        for (double x = start_pt.x; x <= end_pt.x; ++x)
            points.emplace_back(x, pt1.y);
    }
    // otherwise
    else
    {
        double k = static_cast<double>(y_err) / x_err;

        // find the longer axis
        if (std::abs(x_err) > std::abs(y_err))
        {
            if (x_err > 0)
            {
                start_pt = pt1;
                end_pt = pt2;
            }
            else
            {
                start_pt = pt2;
                end_pt = pt1;
            }

            for (double x = start_pt.x; x <= end_pt.x; ++x)
                points.emplace_back(x, pt1.y + (x - pt1.x) * k);
        }
        else
        {
            if (y_err > 0)
            {
                start_pt = pt1;
                end_pt = pt2;
            }
            else
            {
                start_pt = pt2;
                end_pt = pt1;
            }

            for (double y = start_pt.y; y <= end_pt.y; ++y)
                points.emplace_back((y - pt1.y) / k + pt1.x, y);
        }
    }

    return points;
}

std::vector<LLet::lanelet_ptr_t> RoadMap::OccupiedLanelet(CartCooridnate pos)
{
    point_with_id_t geo_pt = coordinate_.CreateLaneletPoint(pos);
    BoundingBox world(geo_pt);

    auto lanelets = lanelet_map_->query(world);
    std::cout << "Number of lanelets found at (" << pos.x << " , " << pos.y << ") : " << lanelets.size() << std::endl;

    return lanelets;
}