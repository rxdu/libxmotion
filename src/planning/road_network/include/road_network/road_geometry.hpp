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

class PolyLine
{
  public:
    PolyLine() = default;

    void AddPoint(double x, double y);

    std::vector<PolyLinePoint> points_;
    void PrintPoints() const;

  private:
    int32_t point_num_ = 0;
};

////////////////////////////////////////////////////////////

class Polygon
{
  public:
    Polygon() = default;
    Polygon(const std::vector<PolyLinePoint> &points);

    void SetBoundary(const std::vector<PolyLinePoint> &points);
    // Note: this check only works with convex polygon
    bool IsInside(const PolyLinePoint &pt) const;

  private:
    // std::vector<PolyLinePoint> boundary_points_;
    std::vector<PolyLineSegment> boundaries_;
};
}

#endif /* ROAD_GEOMETRY_HPP */
