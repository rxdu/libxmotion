/*
 * polyline.hpp
 *
 * Created on: Aug 09, 2018 06:42
 * Description: polyline wrapper around CGAL
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef POLYLINE_HPP
#define POLYLINE_HPP

#include <vector>
#include <limits>

#include "geometry/simple_point.hpp"

namespace rnav {
class LineSegment {
 public:
  LineSegment() = default;
  LineSegment(const SimplePoint2& pt1, const SimplePoint2& pt2);

  bool Intersect(const LineSegment& other) const;

 private:
  SimplePoint2 pt1_;
  SimplePoint2 pt2_;
};

class Polyline {
 public:
  Polyline() = default;
  explicit Polyline(const std::vector<SimplePoint2>& pts);

  typedef std::vector<SimplePoint2>::iterator point_iterator;
  typedef std::vector<SimplePoint2>::const_iterator point_const_iterator;
  point_iterator point_begin() { return points_.begin(); }
  point_iterator point_end() { return points_.end(); }
  point_const_iterator point_begin() const { return points_.begin(); }
  point_const_iterator point_end() const { return points_.end(); }

  void AddPoint(double x, double y);
  void AddPoint(SimplePoint2 pt);
  void SetPoints(std::vector<SimplePoint2> pts) { points_ = pts; }

  bool Intersect(const Polyline& other) const;

  int32_t GetPointNumer() const { return points_.size(); }
  SimplePoint2 GetPoint(std::size_t i) const;
  std::vector<SimplePoint2> GetPoints() const { return points_; }

  inline double GetMinX() const { return xmin_; }
  inline double GetMaxX() const { return xmax_; }
  inline double GetMinY() const { return ymin_; }
  inline double GetMaxY() const { return ymax_; }

  Polyline SeriesConcatenate(const Polyline& other);
  Polyline operator+(const Polyline& other);

  // formatted output
  void PrintInfo() const;

 private:
  std::vector<SimplePoint2> points_;

  double xmin_ = std::numeric_limits<double>::max();
  double xmax_ = std::numeric_limits<double>::min();
  double ymin_ = std::numeric_limits<double>::max();
  double ymax_ = std::numeric_limits<double>::min();

  void UpdateXYMinMax(double x, double y);
};
}  // namespace rnav

#endif /* POLYLINE_HPP */
