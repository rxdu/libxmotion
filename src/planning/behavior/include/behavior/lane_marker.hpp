/* 
 * lane_marker.hpp
 * 
 * Created on: Mar 18, 2018 11:26
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LANE_MARKER_HPP
#define LANE_MARKER_HPP

#include <map>
#include <cstdint>

namespace librav
{
// defined relative to ego vehicle
enum class LaneDirection
{
    Same,
    Opposite,
    Bidirectional
};

class LaneMarker
{
  public:
    LaneDirection direction_;

    // time step, collision node id
    std::map<int32_t, int64_t> conflict_record_;
};
}

#endif /* LANE_MARKER_HPP */
