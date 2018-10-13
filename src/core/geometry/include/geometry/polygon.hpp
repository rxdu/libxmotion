/* 
 * polygon.hpp
 * 
 * Created on: Aug 09, 2018 04:15
 * Description: polygon wrapper around CGAL
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef POLYGON_HPP
#define POLYGON_HPP

#include <iostream>
#include <list>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Partition_is_valid_traits_2.h>
#include <CGAL/polygon_function_objects.h>
#include <CGAL/Polygon_2.h>

#include "geometry/polyline.hpp"
#include "geometry/simple_point.hpp"

namespace librav
{
/// A light wrapper class for CGAL Polygon_2
class Polygon
{
  // Reference:
  //  [1] http://cgal-discuss.949826.n4.nabble.com/CGAL-precondition-error-td4663738.html
  // typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef CGAL::Exact_predicates_exact_constructions_kernel K;
  typedef CGAL::Partition_traits_2<K> Traits;
  typedef CGAL::Is_convex_2<Traits> Is_convex_2;
  typedef Traits::Polygon_2 Polygon_2;
  typedef Traits::Point_2 Point_2;
  typedef std::list<Polygon_2> Polygon_list;
  typedef CGAL::Partition_is_valid_traits_2<Traits, Is_convex_2> Validity_traits;

public:
  Polygon() = default;
  Polygon(std::vector<Point_2> pts);
  Polygon(Polyline left_bound, Polyline right_bound);

  using Point = Point_2;
  typedef Polygon_2::Vertex_iterator point_iterator;
  typedef Polygon_2::Vertex_const_iterator point_const_iterator;
  point_iterator point_begin() { return data_.vertices_begin(); }
  point_iterator point_end() { return data_.vertices_end(); }
  point_const_iterator point_begin() const { return data_.vertices_begin(); }
  point_const_iterator point_end() const { return data_.vertices_end(); }

  std::vector<Polygon> convex_partitions_;

  void AddPoint(double x, double y);
  void AddPoint(Point pt);
  int32_t GetPointNumer() const { return data_.size(); }
  SimplePoint GetPoint(std::size_t i) const;
  void ConvexDecomposition();

  bool CheckInside(Point pt) const;
  int32_t CheckRelativePosition(Point pt) const;
  inline bool IsSimple() const { return data_.is_simple(); }
  inline bool IsConvex() const { return data_.is_convex(); }
  bool Intersect(const Polygon &other) const;
  bool Contain(const Polygon &other) const;

  double GetMinX() const { return CGAL::to_double((*data_.left_vertex()).x()); }
  double GetMaxX() const { return CGAL::to_double((*data_.right_vertex()).x()); }
  double GetMinY() const { return CGAL::to_double((*data_.bottom_vertex()).y()); }
  double GetMaxY() const { return CGAL::to_double((*data_.top_vertex()).y()); }

  Polygon TransformRT(double dx, double dy, double dtheta);
  Polygon TransformTR(double dx, double dy, double dtheta);

  // formated output
  void PrintInfo() const;

private:
  Polygon_2 data_;
  std::list<Polygon_2> partitions_;
  Traits partition_traits_;
  Validity_traits validity_traits_;

  Polygon(Polygon_2 internal_data) : data_(internal_data) {}
};
} // namespace librav

#endif /* POLYGON_HPP */
