/*
 * polygon.cpp
 *
 * Created on: Aug 09, 2018 04:16
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "geometry/polygon.hpp"

#include <iterator>
#include <cassert>
#include <cmath>

namespace xmotion {
Polygon::Polygon(const std::vector<SimplePoint2> &pts) : points_(pts) {
  for (const auto &pt : points_) {
    UpdateXYMinMax(pt.x(), pt.y());
  }
}

void Polygon::AddPoint(double x, double y) {
  points_.emplace_back(x, y);
  UpdateXYMinMax(x, y);
}

void Polygon::AddPoint(SimplePoint2 pt) {
  points_.push_back(pt);
  UpdateXYMinMax(pt.x(), pt.y());
}

SimplePoint2 Polygon::GetPoint(std::size_t i) const {
  assert(i < points_.size());
  return points_[i];
};

bool Polygon::IsSimple() const { return false; }

bool Polygon::IsConvex() const { return false; }

bool Polygon::CheckInside(SimplePoint2 pt) const {
  // TODO
  //   if (CGAL::bounded_side_2(data_.vertices_begin(), data_.vertices_end(),
  //   pt,
  //                            K()) == CGAL::ON_BOUNDED_SIDE)
  //     return true;
  return false;
}

int32_t Polygon::CheckRelativePosition(SimplePoint2 pt) const {
  // TODO
  //   switch (CGAL::bounded_side_2(data_.vertices_begin(),
  //   data_.vertices_end(), pt,
  //                                K())) {
  //     case CGAL::ON_BOUNDED_SIDE:
  //       return 1;
  //     case CGAL::ON_BOUNDARY:
  //       return 0;
  //     case CGAL::ON_UNBOUNDED_SIDE:
  //       return -1;
  //   }
  return 0;
}

bool Polygon::Intersect(const Polygon &other) const {
  //   return CGAL::do_intersect(data_, other.data_);
  // TODO
  return false;
}

bool Polygon::Contain(const Polygon &other) const {
  for (auto it = other.point_begin(); it != other.point_end(); ++it) {
    if (!CheckInside(*it)) return false;
  }
  return true;
}

void Polygon::ConvexDecomposition() {
  // TODO
  //   convex_partitions_.clear();

  //   if (data_.is_convex()) convex_partitions_.push_back(Polygon(data_));

  //   CGAL::optimal_convex_partition_2(data_.vertices_begin(),
  //   data_.vertices_end(),
  //                                    std::back_inserter(partitions_),
  //                                    partition_traits_);

  //   assert(CGAL::partition_is_valid_2(data_.vertices_begin(),
  //                                     data_.vertices_end(),
  //                                     partitions_.begin(),
  //                                     partitions_.end(),
  //                                     validity_traits_));

  //   for (auto it = partitions_.begin(); it != partitions_.end(); ++it)
  //     convex_partitions_.push_back(Polygon(*it));
}

void Polygon::UpdateXYMinMax(double x, double y) {
  if (x < xmin_) xmin_ = x;
  if (x > xmax_) xmax_ = x;
  if (y < ymin_) ymin_ = y;
  if (y > ymax_) ymax_ = y;
}

Polygon Polygon::TransformRT(double dx, double dy, double dtheta) {
  Polygon new_polygon;
  for (const auto &pt : points_) {
    double x = pt.x() * std::cos(dtheta) - pt.y() * std::sin(dtheta) + dx;
    double y = pt.x() * std::sin(dtheta) + pt.y() * std::cos(dtheta) + dy;
    new_polygon.AddPoint(x, y);
  }
  return new_polygon;
}

Polygon Polygon::TransformTR(double dx, double dy, double dtheta) {
  Polygon new_polygon;
  for (const auto &pt : points_) {
    double x =
        (pt.x() + dx) * std::cos(dtheta) - (pt.y() + dy) * std::sin(dtheta);
    double y =
        (pt.x() + dx) * std::sin(dtheta) + (pt.y() + dy) * std::cos(dtheta);
    new_polygon.AddPoint(x, y);
  }
  return new_polygon;
}

void Polygon::PrintInfo() const {
  std::cout << "Polygon with " << points_.size() << " points" << std::endl;
  for (const auto &pt : points_) {
    std::cout << "- (" << pt << ")" << std::endl;
  }
}

//---------------------------------------------------------------------------//

#ifdef ENABLE_VISUAL
using namespace quickviz;

void DrawPolygon(CvCanvas &canvas, const Polygon &polygon, bool show_dot,
                 cv::Scalar ln_color, int32_t thickness) {
  std::size_t pt_num = polygon.GetPointNumer();

  if (pt_num < 3) return;

  for (std::size_t i = 0; i < pt_num - 1; ++i) {
    CPoint pt1(polygon.GetPoint(i).x(), polygon.GetPoint(i).y());
    CPoint pt2(polygon.GetPoint(i + 1).x(), polygon.GetPoint(i + 1).y());
    canvas.DrawLine(pt1, pt2, ln_color, thickness);
  }
  CPoint last_pt(polygon.GetPoint(pt_num - 1).x(),
                 polygon.GetPoint(pt_num - 1).y());
  CPoint first_pt(polygon.GetPoint(0).x(), polygon.GetPoint(0).y());
  canvas.DrawLine(last_pt, first_pt, ln_color, thickness);

  if (show_dot) {
    for (std::size_t i = 0; i < pt_num; ++i)
      canvas.DrawPoint({polygon.GetPoint(i).x(), polygon.GetPoint(i).y()}, 1,
                       CvColors::red_color);
  }
}

void FillPolygon(CvCanvas &canvas, const Polygon &polygon, bool show_dot,
                 cv::Scalar fill_color, cv::Scalar ln_color,
                 int32_t thickness) {
  std::size_t pt_num = polygon.GetPointNumer();

  if (pt_num < 3) return;

  std::vector<CPoint> pts;
  for (int i = 0; i < polygon.GetPointNumer(); ++i)
    pts.emplace_back(polygon.GetPoint(i).x(), polygon.GetPoint(i).y());

  canvas.FillPoly(pts, fill_color);
}
#endif
}  // namespace xmotion
