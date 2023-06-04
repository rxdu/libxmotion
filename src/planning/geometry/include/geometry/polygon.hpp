/*
 * polygon.hpp
 *
 * Created on: Aug 09, 2018 04:15
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef POLYGON_HPP
#define POLYGON_HPP

#include <iostream>
#include <list>
#include <vector>

#include "geometry/simple_point.hpp"

#ifdef ENABLE_VISUAL
#include "cvdraw/cvdraw.hpp"
#endif

namespace xmotion {
class Polygon {
 public:
  Polygon() = default;
  explicit Polygon(const std::vector<SimplePoint2> &pts);

  typedef std::vector<SimplePoint2>::iterator point_iterator;
  typedef std::vector<SimplePoint2>::const_iterator point_const_iterator;
  point_iterator point_begin() { return points_.begin(); }
  point_iterator point_end() { return points_.end(); }
  point_const_iterator point_begin() const { return points_.begin(); }
  point_const_iterator point_end() const { return points_.end(); }

  void AddPoint(double x, double y);
  void AddPoint(SimplePoint2 pt);
  void SetPoints(std::vector<SimplePoint2> pts) { points_ = pts; }

  int32_t GetPointNumer() const { return points_.size(); }
  SimplePoint2 GetPoint(std::size_t i) const;
  std::vector<SimplePoint2> GetPoints() const { return points_; }

  // TODO
  //---------------------------------------
  bool IsSimple() const;
  bool IsConvex() const;
  void ConvexDecomposition();

  bool CheckInside(SimplePoint2 pt) const;
  int32_t CheckRelativePosition(SimplePoint2 pt) const;

  bool Intersect(const Polygon &other) const;
  bool Contain(const Polygon &other) const;
  //---------------------------------------

  double GetMinX() const { return xmin_; }
  double GetMaxX() const { return xmax_; }
  double GetMinY() const { return ymin_; }
  double GetMaxY() const { return ymax_; }

  /// Polygon tranformation: rotate first, then translate
  Polygon TransformRT(double dx, double dy, double dtheta);
  /// Polygon tranformation: translate first, then rotate
  Polygon TransformTR(double dx, double dy, double dtheta);

  // formatted output
  void PrintInfo() const;

 private:
  std::vector<SimplePoint2> points_;
  std::vector<Polygon> convex_partitions_;

  double xmin_ = std::numeric_limits<double>::max();
  double xmax_ = std::numeric_limits<double>::min();
  double ymin_ = std::numeric_limits<double>::max();
  double ymax_ = std::numeric_limits<double>::min();

  void UpdateXYMinMax(double x, double y);
};

#ifdef ENABLE_VISUAL
void DrawPolygon(CvCanvas &canvas, const Polygon &polygon,
                 bool show_dot = false,
                 cv::Scalar ln_color = CvColors::blue_color,
                 int32_t thickness = 1);
void FillPolygon(CvCanvas &canvas, const Polygon &polygon,
                 bool show_dot = false,
                 cv::Scalar fill_color = CvColors::aoi_color,
                 cv::Scalar ln_color = CvColors::blue_color,
                 int32_t thickness = 1);
#endif
}  // namespace xmotion

#endif /* POLYGON_HPP */
