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
    coordinate_.SetOrigin(world_origin_);

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
}

void RoadMap::GenerateDenseGrid(int32_t pixel_per_meter)
{
    int32_t x_size, y_size;

    // add 1 to each dimension to compensate for the rounding
    x_size = static_cast<int32_t>(x_max_ * pixel_per_meter) + 1;
    y_size = static_cast<int32_t>(y_max_ * pixel_per_meter) + 1;

    coordinate_.SetDenseGridSize(x_size, y_size, pixel_per_meter);
    dense_grid_ = std::make_unique<DenseGrid>(x_size, y_size);

    std::cout << "dense grid size: " << x_size << " , " << y_size << " on region " << x_min_ << " , " << x_max_ << " , " << y_min_ << " , " << y_max_ << std::endl;

    double resolution = 1.0 / pixel_per_meter / 10.0;
    for (auto &bound : lane_bounds_)
    {
        // std::cout << "lanelet id: " << bound.first << std::endl;
        std::vector<DenseGridPixel> points_left = GenerateLanePoints(bound.second.first, resolution);
        std::vector<DenseGridPixel> points_right = GenerateLanePoints(bound.second.second, resolution);
        for (const auto &pt : points_left)
            dense_grid_->SetValueAtCoordinate(pt.x, pt.y, 1.0);
        for (const auto &pt : points_right)
            dense_grid_->SetValueAtCoordinate(pt.x, pt.y, 1.0);
    }
}

std::vector<DenseGridPixel> RoadMap::GenerateLanePoints(const PolyLine &line, double resolution)
{
    std::vector<DenseGridPixel> interpolated_points;

    for (int i = 0; i < line.points_.size() - 1; ++i)
    {
        std::vector<DenseGridPixel> pts = InterpolateGridPixelPoints(line.points_[i], line.points_[i + 1], resolution);

        interpolated_points.insert(interpolated_points.end(), pts.begin(), pts.end());
    }

    return interpolated_points;
}

std::vector<DenseGridPixel> RoadMap::InterpolateGridPixelPoints(PolyLinePoint plt1, PolyLinePoint plt2, double resolution)
{
    std::vector<DenseGridPixel> points;

    double x_err = plt2.x - plt1.x;
    double y_err = plt2.y - plt1.y;

    if (x_err == 0)
    {
        if (y_err > 0)
        {
            for (int32_t y = plt1.y; y <= plt2.y; ++y)
                points.emplace_back(plt1.x, y);
        }
        else
        {
            for (int32_t y = plt1.y; y >= plt2.y; --y)
                points.emplace_back(plt1.x, y);
        }
    }
    else
    {
        double k = y_err / x_err;

        if (x_err > 0)
        {
            for (double x = plt1.x; x <= plt2.x; x = x + resolution)
            {
                DenseGridPixel pt = coordinate_.ConvertToGridPixel(CartCooridnate(x, plt1.y + (x - plt1.x) * k));
                if (pt.x >= 0 && pt.y >= 0)
                    points.push_back(pt);
            }
        }
        else
        {
            for (double x = plt1.x; x >= plt2.x; x = x - resolution)
            {
                DenseGridPixel pt = coordinate_.ConvertToGridPixel(CartCooridnate(x, plt1.y + (x - plt1.x) * k));
                if (pt.x >= 0 && pt.y >= 0)
                    points.push_back(pt);
            }
        }
    }

    return points;
}
