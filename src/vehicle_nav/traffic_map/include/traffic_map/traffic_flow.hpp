/* 
 * traffic_flow.hpp
 * 
 * Created on: Aug 22, 2018 09:34
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAFFIC_FLOW_HPP
#define TRAFFIC_FLOW_HPP

#include "geometry/polygon.hpp"

namespace librav
{
struct VehiclePose
{
    VehiclePose() : x(0), y(0), theta(0) {}
    VehiclePose(double _x, double _y, double _theta) : x(_x), y(_y), theta(_theta) {}

    double x;
    double y;
    double theta;
};

///////////////////////////////////////////////////////////////////////////

struct FlowUnit
{
    FlowUnit() = default;
    FlowUnit(VehiclePose p, Polygon poly) : pose(p), footprint(poly) {}
    explicit FlowUnit(Polygon poly) : footprint(poly) {}

    VehiclePose pose;
    Polygon footprint;
    std::string lanelet;

    FlowUnit *parent;
    // a flow unit may have multiple subsequent units at the point
    //  where the lane diverges into multiple lanes
    std::vector<FlowUnit *> children;
};

/// Traffic flow model: one source to multiple sinks
class TrafficFlow
{
  public:
    TrafficFlow() = default;
    explicit TrafficFlow(std::vector<std::vector<std::string>> subflows);

    FlowUnit *root_;

  private:
    std::vector<std::vector<std::string>> subflows_;

    void BuildTree();
};
} // namespace librav

#endif /* TRAFFIC_FLOW_HPP */
