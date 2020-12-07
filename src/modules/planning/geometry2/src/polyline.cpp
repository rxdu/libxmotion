/*
 * polyline.cpp
 *
 * Created on: Aug 09, 2018 06:45
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "geometry/polyline.hpp"

#include <cmath>
#include <cassert>
#include <iostream>

namespace robotnav {

namespace {
bool OnSegment(SimplePoint2 p_i, SimplePoint2 p_j, SimplePoint2 p_k) {
  if (p_k.x() >= std::min(p_i.x(), p_j.x()) &&
      p_k.x() <= std::max(p_i.x(), p_j.x()) &&
      p_k.y() >= std::min(p_i.y(), p_j.y()) &&
      p_k.y() <= std::max(p_i.y(), p_j.y()))
    return true;
  return false;
}

int32_t CalculateDirection(SimplePoint2 p_i, SimplePoint2 p_j, SimplePoint2 p_k) {
  Eigen::Vector2d pt1 = p_k - p_i;
  Eigen::Vector2d pt2 = p_j - p_i;
  return pt1.x() * pt2.y() - pt1.y() * pt1.x();
}
}  // namespace

LineSegment::LineSegment(const SimplePoint2 &pt1, const SimplePoint2 &pt2)
    : pt1_(pt1), pt2_(pt2) {}

// Reference:
// [1] Cormen, T. H., Leiserson, C. E., Rivest, R. L., & Stein, C. (n.d.).
// Introduction to algorithms (3rd ed.). The MIT Press. Chapter 33
// [2]
// https://algorithmtutor.com/Computational-Geometry/Check-if-two-line-segment-intersect/
bool LineSegment::Intersect(const LineSegment &other) const {
  auto p1 = this->pt1_;
  auto p2 = this->pt2_;
  auto p3 = other.pt1_;
  auto p4 = other.pt2_;
  int32_t d1 = CalculateDirection(p3, p4, p1);
  int32_t d2 = CalculateDirection(p3, p4, p2);
  int32_t d3 = CalculateDirection(p1, p2, p3);
  int32_t d4 = CalculateDirection(p1, p2, p4);
  if ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0) || (d3 > 0 && d4 < 0) ||
      (d3 < 0 && d4 > 0))
    return true;
  else if (d1 == 0 && OnSegment(p3, p4, p1))
    return true;
  else if (d2 == 0 && OnSegment(p3, p4, p2))
    return true;
  else if (d3 == 0 && OnSegment(p1, p2, p3))
    return true;
  else if (d4 == 0 && OnSegment(p1, p2, p4))
    return true;
  return false;
}

//---------------------------------------------------------------------------//

Polyline::Polyline(const std::vector<SimplePoint2> &pts) {
  points_ = pts;
  for (auto &pt : pts) {
    UpdateXYMinMax(pt.x(), pt.y());
  }
}

void Polyline::AddPoint(double x, double y) {
  UpdateXYMinMax(x, y);
  points_.push_back(SimplePoint2(x, y));
}

void Polyline::AddPoint(SimplePoint2 pt) {
  UpdateXYMinMax(pt.x(), pt.y());
  points_.push_back(pt);
}

bool Polyline::Intersect(const Polyline &other) const {
  for (int32_t i = 0; i < GetPointNumer() - 1; ++i) {
    LineSegment seg1(points_[i], points_[i + 1]);
    for (int32_t j = 0; j < other.GetPointNumer() - 1; ++j) {
      LineSegment seg2(other.GetPoints()[j], other.GetPoints()[j + 1]);
      if (seg1.Intersect(seg2)) return true;
    }
  }
  return false;
}

SimplePoint2 Polyline::GetPoint(std::size_t i) const {
  assert(i < points_.size());
  return points_[i];
}

// Example:
// Polyline1 = {pt1, pt2, pt3}, Polyline2 = {pt3, pt4, pt5}
// Polyline1.SeriesConcatenate(Polyline2) = {pt1, pt2, pt3, pt4, pt5}
Polyline Polyline::SeriesConcatenate(const Polyline &other) {
  assert(other.GetPointNumer() > 0);

  std::vector<SimplePoint2> points;
  points.insert(points.end(), points_.begin(), points_.end());

  auto other_pts = other.GetPoints();
  if (points_.empty())
    points.insert(points.end(), other_pts.begin(), other_pts.end());
  else
    points.insert(points.end(), other_pts.begin() + 1, other_pts.end());

  return Polyline(points);
}

Polyline Polyline::operator+(const Polyline &other) {
  std::vector<SimplePoint2> points;
  points.insert(points.end(), points_.begin(), points_.end());

  auto other_pts = other.GetPoints();
  points.insert(points.end(), other_pts.begin(), other_pts.end());

  return Polyline(points);
}

void Polyline::PrintInfo() const {
  std::cout << "Polyline with " << points_.size() << " points" << std::endl;
  for (auto &pt : points_) std::cout << "- (" << pt << ")" << std::endl;
}

void Polyline::UpdateXYMinMax(double x, double y) {
  if (x < xmin_) xmin_ = x;
  if (x > xmax_) xmax_ = x;
  if (y < ymin_) ymin_ = y;
  if (y > ymax_) ymax_ = y;
}
}  // namespace robotnav