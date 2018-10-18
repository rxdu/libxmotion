/* 
 * traffic_segment.hpp
 * 
 * Created on: Oct 18, 2018 08:55
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAFFIC_SEGMENT_HPP
#define TRAFFIC_SEGMENT_HPP

#include <string>
#include <vector>

namespace librav
{
class TrafficSegment
{
  public:
    TrafficSegment() = default;
    TrafficSegment(std::string first_lane, std::string last_lane);

    std::string first_lane_;
    std::string last_lane_;
};
} // namespace librav

#endif /* TRAFFIC_SEGMENT_HPP */
