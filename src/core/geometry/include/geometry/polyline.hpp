/* 
 * polyline.hpp
 * 
 * Created on: Aug 09, 2018 06:42
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef POLYLINE_HPP
#define POLYLINE_HPP

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

public:
  Polyline() = default;
  Polyline(std::vector<Point_2> pts);

  using Point = Point_2;
  using Line = Line_2;

  void AddPoint(double x, double y);
  void AddPoint(Point pt);
  void SetPoints(std::vector<Point_2> pts) { points_ = pts; }

  std::size_t GetPointNumer() const { return points_.size(); }
  SimplePoint GetPoint(std::size_t i) const;
  std::vector<Point_2> GetPoints() const { return points_; }
  std::vector<SimplePoint> GetSimplePoints() const;

  inline double GetMinX() const { return xmin_; }
  inline double GetMaxX() const { return xmax_; }
  inline double GetMinY() const { return ymin_; }
  inline double GetMaxY() const { return ymax_; }

  void PrintInfo();

private:
  std::vector<Point_2> points_;

  double xmin_;
  double xmax_;
  double ymin_;
  double ymax_;

  void UpdateXYMinMax(double x, double y);
};
} // namespace librav

#endif /* POLYLINE_HPP */
