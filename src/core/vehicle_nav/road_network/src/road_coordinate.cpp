/* 
 * road_coordinate.cpp
 * 
 * Created on: Apr 03, 2018 18:08
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "road_network/road_coordinate.hpp"

#include <iostream>

using namespace librav;

void RoadCoordinateFrame::SetOrigin(LLet::point_with_id_t origin)
{
    origin_ = origin;
    local_coordinate_.set_origin(std::get<LLet::LAT>(origin), std::get<LLet::LON>(origin));
}

void RoadCoordinateFrame::SetDenseGridSize(int32_t x, int32_t y, int32_t ppm)
{
    grid_size_x_ = x;
    grid_size_y_ = y;
    pixel_per_meter_ = ppm;
}

LLet::point_with_id_t RoadCoordinateFrame::CreateLaneletPoint(CartCooridnate input)
{
    double output_lat, output_lon;
    std::tie(output_lat, output_lon) = local_coordinate_.xy2ll(input.x, input.y);
    return std::make_tuple(output_lat, output_lon, -1);
}

GeoCoordinate RoadCoordinateFrame::ConvertToGeographic(CartCooridnate input)
{
    double output_lat, output_lon;
    std::tie(output_lat, output_lon) = local_coordinate_.xy2ll(input.x, input.y);
    return GeoCoordinate(output_lat, output_lon);
}

CartCooridnate RoadCoordinateFrame::ConvertToCartesian(LLet::point_with_id_t input)
{
    std::pair<double, double> output = LLet::vec(origin_, input);
    return CartCooridnate(output.first, output.second);
}

CartCooridnate RoadCoordinateFrame::ConvertToCartesian(GeoCoordinate input)
{
    double output_x, output_y;
    std::tie(output_x, output_y) = local_coordinate_.ll2xy(input.latitude, input.longitude);
    return CartCooridnate(output_x, output_y);
}

DenseGridPixel RoadCoordinateFrame::ConvertToGridPixel(CartCooridnate input)
{
    int32_t x = input.x * pixel_per_meter_;
    int32_t y = grid_size_y_ - input.y * pixel_per_meter_;

    if (x < 0)
        x = 0;
    if (y < 0)
        y = 0;
    if (x >= grid_size_x_)
        x = grid_size_x_ - 1;
    if (y >= grid_size_y_)
        y = grid_size_y_ - 1;
    return DenseGridPixel(x, y);

    // return DenseGridPixel(input.x * pixel_per_meter_, grid_size_y_ - input.y * pixel_per_meter_);
}

CartCooridnate RoadCoordinateFrame::ConvertToCartesian(DenseGridPixel input)
{
    return CartCooridnate(static_cast<double>(input.x) / pixel_per_meter_, static_cast<double>(grid_size_y_ - input.y) / pixel_per_meter_);
}
