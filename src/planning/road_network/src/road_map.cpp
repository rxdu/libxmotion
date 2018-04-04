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

    // auto bound = lane_bounds_[-39200];
    // std::vector<DenseGridPixel> points_left = GenerateLanePoints(bound.first, resolution);
    // // std::vector<DenseGridPixel> points_right = GenerateLanePoints(bound.second, resolution);
    // for (const auto &pt : points_left)
    //     dense_grid_->SetValueAtCoordinate(pt.x, pt.y, 1.0);
    // // for (const auto &pt : points_right)
    // //     dense_grid_->SetValueAtCoordinate(pt.x, pt.y, 1.0);
}

std::vector<DenseGridPixel> RoadMap::GenerateLanePoints(const PolyLine &line, double resolution)
{
    std::vector<DenseGridPixel> grid_points;
    std::vector<DenseGridPixel> interpolated_points;

    // for (int i = 0; i < line.points_.size() - 1; ++i)
    // {
    //     std::vector<DenseGridPixel> pts = InterpolateGridPixelPoints(line.points_[i], line.points_[i + 1], resolution);

    //     interpolated_points.insert(interpolated_points.end(), pts.begin(), pts.end());
    // }

    for (const auto &pt : line.points_)
    // for (int i = 0; i < 3; ++i)
    {
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

std::vector<DenseGridPixel> RoadMap::InterpolateGridPixelPoints(DenseGridPixel pt1, DenseGridPixel pt2)
{
    std::vector<DenseGridPixel> points;

    int32_t x_err = pt2.x - pt1.x;
    int32_t y_err = pt2.y - pt1.y;

    std::cout << "xerr: " << x_err << " , y_err: " << y_err << std::endl;

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
    std::cout << "Number of lanelets found: " << lanelets_.size() << std::endl;

    return lanelets;
}