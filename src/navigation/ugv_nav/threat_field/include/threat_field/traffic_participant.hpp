/* 
 * traffic_participant.hpp
 * 
 * Created on: Mar 08, 2018 15:24
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAFFIC_PARTICIPANT_HPP
#define TRAFFIC_PARTICIPANT_HPP

#include <cstdint>
#include <functional>

namespace librav
{

/*
 * Coordinate System:
 *
 *		y
 *		^
 *		|         
 *		|         
 *		|         
 *		| 
 *		|         
 *		|         
 *		|  
 *		o ------------------> x         
 *
 * The coordinate system of the traffic participant is 
 *  the same with that of the road map.
 * 
 */
struct TrafficParticipant
{
  TrafficParticipant() : position_x(0), position_y(0), velocity_x(0), velocity_y(0) {}
  TrafficParticipant(double px, double py, double vx, double vy) : position_x(px), position_y(py), velocity_x(vx), velocity_y(vy) {}

  int32_t id;
  double position_x;
  double position_y;
  double velocity_x;
  double velocity_y;
  double time_stamp;

  std::function<double(double, double)> threat_func;

  double GetThreatValue(double x, double y) { return threat_func(x, y); }
};
} // namespace librav

#endif /* TRAFFIC_PARTICIPANT_HPP */
