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
#include "traffic_map/traffic_elements.hpp"

namespace librav
{

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
    TrafficFlow(std::vector<TrafficChannel> channels);

    FlowUnit *root_;

  private:
    std::vector<TrafficChannel> channels_;
};
} // namespace librav

#endif /* TRAFFIC_FLOW_HPP */
