/* 
 * road_geometry.hpp
 * 
 * Created on: Apr 05, 2018 13:35
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef ROAD_GEOMETRY_HPP
#define ROAD_GEOMETRY_HPP

#include <cstdint>
#include <vector>

#include <Eigen/Dense>

namespace librav
{
struct PolyLinePoint
{
  PolyLinePoint(double px = 0, double py = 0) : x(px), y(py){};

  double x;
  double y;
};

class PolyLineSegment
{
public:
  PolyLineSegment(PolyLinePoint pt1, PolyLinePoint pt2);

  bool IsOnTheRightSide(const PolyLinePoint &pt) const;

private:
  PolyLinePoint start_point_;
  PolyLinePoint end_point_;

  Eigen::Vector3d base_vec_;
};

////////////////////////////////////////////////////////////

struct PolyLine
{
  std::vector<PolyLinePoint> points_;

  void AddPoint(double x, double y);
  void SetPoints(std::vector<PolyLinePoint> pts) { points_ = pts; }

  void PrintPoints() const;
};

////////////////////////////////////////////////////////////

class Polygon
{
public:
  Polygon() = default;
  Polygon(const std::vector<PolyLinePoint> &points);

  void SetBoundary(const std::vector<PolyLinePoint> &points);

  // Note: this function works with only convex polygons
  bool InsideConvexPolygon(const PolyLinePoint &pt);
  // Note: this function works with both convex and concave polygons
  bool InsidePolygon(const PolyLinePoint &p);

private:
  std::vector<PolyLinePoint> boundary_points_;
  std::vector<PolyLineSegment> boundaries_;
  double Angle2D(double x1, double y1, double x2, double y2);
};
} // namespace librav

#endif /* ROAD_GEOMETRY_HPP */
