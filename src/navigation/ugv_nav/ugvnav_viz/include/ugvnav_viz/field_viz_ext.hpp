/* 
 * field_viz_ext.hpp
 * 
 * Created on: Aug 13, 2018 01:11
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef FIELD_VIZ_EXT_HPP
#define FIELD_VIZ_EXT_HPP

#include <vector>
#include <memory>

#include "geometry/polygon.hpp"

// #include "road_map/road_map.hpp"
#include "threat_field/traffic_participant.hpp"
#include "threat_field/collision_field.hpp"

namespace librav
{
namespace LightViz
{
void ShowTrafficParticipant(std::shared_ptr<TrafficParticipant> participant, bool show_wp = true, int32_t pixel_per_unit = 10, std::string window_name = "Field Image", bool save_img = false);

void ShowCollisionField(std::shared_ptr<CollisionField> cfield,
                        bool show_wp = true, int32_t pixel_per_unit = 10, std::string window_name = "Field Image", bool save_img = false);

void ShowTrafficParticipantThreat(std::shared_ptr<TrafficParticipant> participant, const Polygon &polygon, int32_t pixel_per_unit = 10, std::string window_name = "Vehicle Image", bool save_img = false);
} // namespace LightViz
} // namespace librav

#endif /* FIELD_VIZ_EXT_HPP */
