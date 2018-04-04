/* 
 * road_coordinate.hpp
 * 
 * Created on: Apr 03, 2018 17:18
 * Description: it's assumed that the map is defined in north america and
 *          the origin is defined at the south-west corner on the map
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef ROAD_COORDINATE_HPP
#define ROAD_COORDINATE_HPP

#include <liblanelet/Lanelet.hpp>
#include <liblanelet/lanelet_point.hpp>

#include "road_network/polyline.hpp"

namespace librav
{
/*
 *  Internal Coordinate System:
 * 
 *		y
 *		^
 *		|         
 *		|         
 *		|         
 *		| 
 *		|         
 *		|         
 *		|  
 *		o ------------------> x                
 *		
 */
template <typename T>
struct XYCooridnate
{
  XYCooridnate(T px = 0, T py = 0) : x(px), y(py){};

  T x;
  T y;
};

using CartCooridnate = XYCooridnate<double>;
using DenseGridPixel = XYCooridnate<int32_t>;

struct GeoCoordinate
{
  GeoCoordinate(double lat = 0, double lon = 0) : latitude(lat), longitude(lon){};

  double latitude;
  double longitude;
};

/////////////////////////////////////////////////////////////

class RoadCoordinateFrame
{
public:
  RoadCoordinateFrame() = default;

  void SetOrigin(LLet::point_with_id_t origin);
  void SetDenseGridSize(int32_t x, int32_t y, int32_t ppm);

  LLet::point_with_id_t CreateLaneletPoint(CartCooridnate input);

  CartCooridnate ConvertToCartesian(LLet::point_with_id_t input);

  GeoCoordinate ConvertToGeographic(CartCooridnate input);
  CartCooridnate ConvertToCartesian(GeoCoordinate input);

  DenseGridPixel ConvertToGridPixel(CartCooridnate input);
  CartCooridnate ConvertToCartesian(DenseGridPixel input);

private:
  LLet::point_with_id_t origin_;
  LocalGeographicCS local_coordinate_;
  int32_t pixel_per_meter_;
  int32_t grid_size_x_;
  int32_t grid_size_y_;
};
}

#endif /* ROAD_COORDINATE_HPP */
