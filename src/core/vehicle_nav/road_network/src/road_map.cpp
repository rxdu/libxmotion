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

#include "stopwatch/stopwatch.h"

using namespace librav;
using namespace LLet;

RoadMap::RoadMap(std::string map_osm, int32_t ppm) : pixel_per_meter_(ppm)
{
    if (!map_osm.empty())
        LoadMapFile(map_osm, pixel_per_meter_);
}

void RoadMap::LoadMapFile(std::string map_file, int32_t ppm)
{
    if (map_file.empty())
        return;

    pixel_per_meter_ = ppm;

    // create lanelet map from osm file
    lanelet_map_ = std::make_unique<LaneletMap>(map_file);
    std::cout << "Map loaded: " << map_file << std::endl;

    // load all lanelets in the map
    BoundingBox world(std::make_tuple(-180, -180, 180, 180));
    lanelets_ = lanelet_map_->query(world);
    std::cout << "Number of lanelets found: " << lanelets_.size() << std::endl;

    // find reference node with name "origin" and use it as the origin of local reference frame
    bool origin_found = lanelet_map_->get_local_reference_frame_origin(&world_origin_);

    if (!origin_found)
        return;

    // continue setup if origin reference point is found
    coordinate_.SetOrigin(world_origin_);

    // extract lane bounds from all lanelets
    std::vector<double> x_coordinates;
    std::vector<double> y_coordinates;
    for (auto &lanelet : lanelets_)
    {
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

    // generate dense grid masks for planning
    GenerateMapMasks();

    // set loaded flag
    map_loaded_ = true;
}

void RoadMap::GenerateMapMasks()
{
    // add 1 to each dimension to compensate for the rounding
    grid_size_x_ = static_cast<int32_t>(x_max_ * pixel_per_meter_) + 1;
    grid_size_y_ = static_cast<int32_t>(y_max_ * pixel_per_meter_) + 1;

    coordinate_.SetDenseGridSize(grid_size_x_, grid_size_y_, pixel_per_meter_);

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

        lane_bound_grids_.insert(std::make_pair(bound.first, sub_grid));
    }

    ExtractDrivableAreas();
    ExtractLaneCenterLines();
}

void RoadMap::ExtractDrivableAreas()
{
    /* use annotated reference points */
    std::vector<PolyLinePoint> appr_bound_pts;
    for (auto &bound_pt : lanelet_map_->get_drivable_boundary_points())
    {
        auto cart = coordinate_.ConvertToCartesian(bound_pt);
        appr_bound_pts.emplace_back(cart.x, cart.y);
    }
    // to close the boundary loop
    auto first_cart = coordinate_.ConvertToCartesian(lanelet_map_->get_drivable_boundary_points().front());
    appr_bound_pts.emplace_back(first_cart.x, first_cart.y);
    Polygon appr_polygon(appr_bound_pts);

    /* use lanelet boundary points */
    std::unordered_map<int32_t, Polygon> polygons;
    for (auto &bound : lane_bounds_)
    {
        auto sub_grid = std::make_shared<DenseGrid>(grid_size_x_, grid_size_y_);
        lane_drivable_grids_.insert(std::make_pair(bound.first, sub_grid));

        std::vector<PolyLinePoint> bound_pts;
        for (auto &pt : bound.second.first.points_)
            bound_pts.push_back(pt);
        for (auto rit = bound.second.second.points_.rbegin(); rit != bound.second.second.points_.rend(); ++rit)
            bound_pts.push_back(*rit);
        bound_pts.push_back(bound.second.first.points_.front());

        polygons.insert(std::make_pair(bound.first, Polygon(bound_pts)));
        lane_drivable_grids_.insert(std::make_pair(bound.first, sub_grid));
    }

    stopwatch::StopWatch timer;

    for (int i = 0; i < grid_size_x_; ++i)
        for (int j = 0; j < grid_size_y_; ++j)
        {
            auto pt = coordinate_.ConvertToCartesian(DenseGridPixel(i, j));

            if (!appr_polygon.InsidePolygon(PolyLinePoint(pt.x, pt.y)))
                continue;

            for (auto &pg : polygons)
            {
                if (pg.second.InsidePolygon(PolyLinePoint(pt.x, pt.y)))
                    lane_drivable_grids_[pg.first]->SetValueAtCoordinate(i, j, 1);
            }
        }

    std::cout << "extract drivable areas in " << timer.toc() << " seconds" << std::endl;

    mask_zero_ = Eigen::MatrixXd::Zero(grid_size_y_, grid_size_x_);
    mask_ones_ = Eigen::MatrixXd::Ones(grid_size_y_, grid_size_x_);
}

void RoadMap::ExtractLaneCenterLines()
{
    // extract center line points
    for (auto &ln : lanelet_map_->get_lane_center_lines())
    {
        PolyLine line;
        for (auto pt : ln.second)
        {
            auto cart = coordinate_.ConvertToCartesian(pt);
            line.AddPoint(cart.x, cart.y);
        }
        lane_center_lines_.insert(std::make_pair(ln.first, line));
    }

    // generate masks
    for (auto &cline : lane_center_lines_)
    {
        auto sub_grid = std::make_shared<DenseGrid>(grid_size_x_, grid_size_y_);
        std::vector<DenseGridPixel> linepts = GenerateLanePoints(cline.second);
        for (const auto &pt : linepts)
            sub_grid->SetValueAtCoordinate(pt.x, pt.y, 1.0);

        lane_centerline_grids_.insert(std::make_pair(GetLaneletIDByCenterLine(cline.first), sub_grid));
    }
}

/// Returns the lanelet id corresponding to the given center line identified by its name
int32_t RoadMap::GetLaneletIDByCenterLine(std::string cl_name)
{
    cl_name.erase(0, 3);
    return ll_id_lookup_[cl_name];
}

std::shared_ptr<DenseGrid> RoadMap::GetFullLaneBoundaryGrid()
{
    auto full_grid = std::make_shared<DenseGrid>(grid_size_x_, grid_size_y_);
    for (auto grid : lane_bound_grids_)
    {
        full_grid->AddGrid(*grid.second.get());
    }
    return full_grid;
}

std::shared_ptr<DenseGrid> RoadMap::GetLaneBoundGrid(std::vector<std::string> lanelets)
{
    auto grid = std::make_shared<DenseGrid>(grid_size_x_, grid_size_y_);
    for (const auto &ll_name : lanelets)
    {
        grid->AddGrid(*lane_bound_grids_[ll_id_lookup_[ll_name]].get());
    }
    return grid;
}

std::shared_ptr<DenseGrid> RoadMap::GetLaneBoundGrid(std::vector<int32_t> lanelets)
{
    auto grid = std::make_shared<DenseGrid>(grid_size_x_, grid_size_y_);
    for (const auto &ll_id : lanelets)
    {
        grid->AddGrid(*lane_bound_grids_[ll_id].get());
    }
    return grid;
}

std::shared_ptr<DenseGrid> RoadMap::GetFullDrivableAreaGrid()
{
    auto grid = std::make_shared<DenseGrid>(grid_size_x_, grid_size_y_);
    Eigen::MatrixXd occupancy_matrix = Eigen::MatrixXd::Zero(grid_size_y_, grid_size_x_);
    for (auto ll : lane_drivable_grids_)
        occupancy_matrix += ll.second->GetGridMatrix(false);
    Eigen::MatrixXd matrix = (occupancy_matrix.array() > 0).select(mask_zero_, mask_ones_);
    grid->SetupGridWithMatrix(matrix);

    return grid;
}

std::shared_ptr<DenseGrid> RoadMap::GetFullCenterLineGrid()
{
    auto grid = std::make_shared<DenseGrid>(grid_size_x_, grid_size_y_);
    Eigen::MatrixXd occupancy_matrix = Eigen::MatrixXd::Zero(grid_size_y_, grid_size_x_);
    for (auto ll : lane_centerline_grids_)
        occupancy_matrix += ll.second->GetGridMatrix(false);
    Eigen::MatrixXd matrix = (occupancy_matrix.array() > 0).select(mask_zero_, mask_ones_);
    grid->SetupGridWithMatrix(matrix);

    return grid;
}

// Reference: http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt
std::shared_ptr<DenseGrid> RoadMap::GetLaneDrivableGrid(std::vector<std::string> lanelets)
{
    auto grid = std::make_shared<DenseGrid>(grid_size_x_, grid_size_y_);
    Eigen::MatrixXd occupancy_matrix = Eigen::MatrixXd::Zero(grid_size_y_, grid_size_x_);
    for (auto ll_name : lanelets)
        occupancy_matrix += lane_drivable_grids_[ll_id_lookup_[ll_name]]->GetGridMatrix(false);
    Eigen::MatrixXd matrix = (occupancy_matrix.array() > 0).select(mask_zero_, mask_ones_);
    grid->SetupGridWithMatrix(matrix);

    return grid;
}

void RoadMap::SetTrafficSinkSource(std::vector<std::string> sink, std::vector<std::string> source)
{
    // TODO: sinks and sources could be identified automatically by checking vertices
    //  in the lanelet graph. A sink/source should have only incoming or outgoing connections
    //  with other lanelets. The lanelet graph is a directed graph and the direction can be
    //  used to check a node is sink or source.
    traffic_sinks_ = sink;
    traffic_sources_ = source;
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
        auto gpt = coordinate_.ConvertToGridPixel(CartCooridnate(pt.x, pt.y));
        grid_points.push_back(gpt);
    }

    for (int i = 0; i < grid_points.size() - 1; ++i)
    {
        std::vector<DenseGridPixel> pts = InterpolateGridPixelPoints(grid_points[i], grid_points[i + 1]);

        interpolated_points.insert(interpolated_points.end(), pts.begin(), pts.end());
    }

    return interpolated_points;
}

// Reference: https://www.redblobgames.com/grids/line-drawing.html
std::vector<DenseGridPixel> RoadMap::InterpolateGridPixelPoints(DenseGridPixel pt1, DenseGridPixel pt2)
{
    std::vector<DenseGridPixel> points;

    int32_t x_err = pt2.x - pt1.x;
    int32_t y_err = pt2.y - pt1.y;

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

PolyLine RoadMap::GetLaneCenterLine(std::string lane_name)
{
    return lane_center_lines_[GetCenterLineNameByLanelet(lane_name)];
}

std::vector<int32_t> RoadMap::OccupiedLanelet(CartCooridnate pos)
{
    point_with_id_t geo_pt = coordinate_.CreateLaneletPoint(pos);
    BoundingBox world(geo_pt);

    auto lanelets = lanelet_map_->query(world);
    // std::cout << "Number of lanelets found at (" << pos.x << " , " << pos.y << ") : " << lanelets.size() << std::endl;
    // return lanelets;

    std::vector<int32_t> ids;
    for (auto ll : lanelets)
        ids.push_back(ll->id());

    return ids;
}