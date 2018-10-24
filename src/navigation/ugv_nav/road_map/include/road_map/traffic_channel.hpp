/* 
 * traffic_channel.hpp
 * 
 * Created on: Oct 18, 2018 08:28
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAFFIC_CHANNEL_HPP
#define TRAFFIC_CHANNEL_HPP

#include <vector>
#include <string>
#include <cstdint>
#include <memory>

#include "geometry/simple_point.hpp"
#include "geometry/polyline.hpp"
#include "geometry/parametric_curve.hpp"
#include "decomp/curvilinear_grid.hpp"

namespace librav
{
class RoadMap;

class TrafficChannel
{
public:
  using GridPointLocal = CurvilinearGrid::GridPoint;

  struct GridPointGlobal
  {
    GridPointGlobal(double _x = 0, double _y = 0, double _theta = 0, double _kappa = 0) : x(_x), y(_y), theta(_theta), kappa(_kappa){};

    double x;
    double y;
    double theta;
    double kappa;
  };

public:
  TrafficChannel(RoadMap *map, std::string src, std::string dst, std::vector<std::string> lanes);
  ~TrafficChannel() = default;

  TrafficChannel(const TrafficChannel &other) = default;
  TrafficChannel(TrafficChannel &&other) = default;
  TrafficChannel &operator=(const TrafficChannel &other) = default;
  TrafficChannel &operator=(TrafficChannel &&other) = default;

  std::string source_;
  std::string sink_;
  std::vector<std::string> lanes_;

  Polyline center_line_;
  ParametricCurve center_curve_;
  std::shared_ptr<CurvilinearGrid> grid_;

  void DiscretizeChannel(double step_t, double step_n, int32_t side_num);

  bool CheckInside(SimplePoint pt);
  CurvilinearGrid::GridPoint ConvertToPathCoordinate(SimplePoint pt);
  SimplePoint ConvertToGlobalCoordinate(CurvilinearGrid::GridPoint pt);

  GridPointGlobal MapLocalPointToGlobal(GridPointLocal pt);

  void PrintInfo();

private:
  RoadMap *road_map_;
  double origin_offset_ = 0.0;

  double GetPointLineDistance(SimplePoint ln_pt1, SimplePoint ln_pt2, SimplePoint pt);
  CurvilinearGrid::GridPoint FindApproximatePoint(SimplePoint pt);
};
} // namespace librav

#endif /* TRAFFIC_CHANNEL_HPP */
