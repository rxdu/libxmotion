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

#include <string>
#include <cstdint>

#include "geometry/polyline.hpp"

namespace librav
{
class TrafficChannel
{
  public:
    TrafficChannel(std::string src, std::string dst) : source(src), sink(dst)
    {
    }

    std::string source;
    std::string sink;

    Polyline center_line;
};
} // namespace librav

#endif /* TRAFFIC_CHANNEL_HPP */
