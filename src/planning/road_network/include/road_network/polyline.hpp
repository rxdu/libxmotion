/* 
 * polyline.hpp
 * 
 * Created on: Apr 03, 2018 15:58
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef POLYLINE_HPP
#define POLYLINE_HPP

#include <cstdint>
#include <vector>
#include <unordered_map>

namespace librav
{
struct PolyLinePoint
{
    PolyLinePoint(double px, double py) : x(px), y(py){};

    double x;
    double y;
};

class PolyLineSegment
{
  public:
    PolyLineSegment(PolyLinePoint pt1, PolyLinePoint pt2);

  private:
    PolyLinePoint start_point_;
    PolyLinePoint end_point_;
};

class PolyLine
{
  public:
    PolyLine() = default;

    void AddPoint(double x, double y);

    std::unordered_map<int32_t, PolyLinePoint> points_;
    void PrintPoints() const;

  private:
    int32_t point_num_ = 0;
};
}

#endif /* POLYLINE_HPP */
