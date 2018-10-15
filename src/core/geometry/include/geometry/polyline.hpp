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

// #include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/polygon_function_objects.h>

#include "geometry/simple_point.hpp"

namespace librav
{
class Polyline
{
  // typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Exact_predicates_exact_constructions_kernel K;
  typedef CGAL::Partition_traits_2<K> Traits;
  typedef Traits::Point_2 Point_2;
  typedef Traits::Line_2 Line_2;
  typedef Traits::Segment_2 Segment_2;

public:
  Polyline() = default;
  Polyline(std::vector<Point_2> pts);
  ~Polyline() = default;

  Polyline(const Polyline& other) = default;
  Polyline(Polyline&& other) = default;
  Polyline& operator=(const Polyline& other) = default;
  Polyline& operator=(Polyline&& other) = default;

  using Point = Point_2;
  using Line = Line_2;
  using Segment = Segment_2;

  typedef std::vector<Point_2>::iterator point_iterator;
  typedef std::vector<Point_2>::const_iterator point_const_iterator;
  point_iterator point_begin() { return points_.begin(); }
  point_iterator point_end() { return points_.end(); }
  point_const_iterator point_begin() const { return points_.begin(); }
  point_const_iterator point_end() const { return points_.end(); }

  void AddPoint(double x, double y);
  void AddPoint(Point pt);
  void SetPoints(std::vector<Point_2> pts) { points_ = pts; }

  bool Intersect(const Polyline &other) const;

  int32_t GetPointNumer() const { return points_.size(); }
  SimplePoint GetPoint(std::size_t i) const;
  std::vector<Point_2> GetPoints() const { return points_; }
  std::vector<SimplePoint> GetSimplePoints() const;

  inline double GetMinX() const { return xmin_; }
  inline double GetMaxX() const { return xmax_; }
  inline double GetMinY() const { return ymin_; }
  inline double GetMaxY() const { return ymax_; }

  Polyline Concatenate(const Polyline &other);
  Polyline operator+(const Polyline &other);

  void PrintInfo();

private:
  std::vector<Point_2> points_;

  double xmin_ = std::numeric_limits<double>::max();
  double xmax_ = std::numeric_limits<double>::min();
  double ymin_ = std::numeric_limits<double>::max();
  double ymax_ = std::numeric_limits<double>::min();

  void UpdateXYMinMax(double x, double y);
};
} // namespace librav

#endif /* POLYLINE_HPP */
