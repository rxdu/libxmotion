/*
 * collision_field.hpp
 *
 * Created on: Nov 17, 2017 11:18
 * Description: This is the field for planning, which may consist of multiple
 *          layers of sub-fields describing different types of collisions
 *
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef COLLISION_FIELD_HPP
#define COLLISION_FIELD_HPP

#include <memory>
#include <vector>
#include <cstdint>
#include <unordered_map>

#include "threat_field/traffic_participant.hpp"
#include "threat_field/threat_distribution.hpp"

#include "threat_field/threat_distribution.hpp"

namespace librav
{
enum class ThreatType : int
{
  Vehicle = 0
};

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
 * The coordinate system of the collision field is the consistent with 
 *  the one for the traffic participants.
 * 
 */

class CollisionField
{
public:
  CollisionField() = delete;
  CollisionField(double xmin, double xmax, double ymin, double ymax);

  double xmin_ = 0;
  double xmax_ = 0;
  double ymin_ = 0;
  double ymax_ = 0;

  void SetSize(double xmin, double xmax, double ymin, double ymax);
  inline double GetMeanX() { return (xmin_ + xmax_) / 2.0; }
  inline double GetMeanY() { return (ymin_ + ymax_) / 2.0; }
  inline double GetSpanX() { return xmax_ - xmin_; }
  inline double GetSpanY() { return ymax_ - ymin_; }

  void AddTrafficParticipant(int32_t id, std::shared_ptr<TrafficParticipant> participant);
  std::shared_ptr<TrafficParticipant> GetTrafficParticipant(int32_t id);
  void RemoveTrafficParticipant(int32_t id);

  std::size_t GetTrafficParticipantNumber() const { return traffic_participants_.size(); };
  double GetCollisionThreat(double x, double y) { return (*this)(x, y); }

  double operator()(double x, double y);

private:
  std::unordered_map<int32_t, std::shared_ptr<TrafficParticipant>> traffic_participants_;
};
} // namespace librav

#endif /* COLLISION_FIELD_HPP */
