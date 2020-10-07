/* 
 * cav_mission_control.hpp
 * 
 * Created on: Dec 09, 2018 22:52
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CAV_MISSION_CONTROL_HPP
#define CAV_MISSION_CONTROL_HPP

#include <string>
#include <memory>

#include "cav_motion/cav_datalink.hpp"
#include "cav_motion/route_planner.hpp"
#include "traffic_map/map_loader.hpp"

namespace autodrive
{
class CAVMissionControl
{
  public:
    CAVMissionControl(std::string map_file);

    bool IsReady();

    // Note: in a complete system, the initial and goal state of the mission should be set
    //      by localization sub-system and user, respectively
    void GenerateMissionInitialState(std::string src, std::string dst, double s, double delta);
    void GenerateMissionGoalState(std::string src, std::string dst, double s, double delta);

    void Run();

  private:
    MapLoader map_loader_;
    std::shared_ptr<RoadMap> road_map_;
    std::shared_ptr<TrafficMap> traffic_map_;
    std::shared_ptr<LCMLink> data_link_;
    std::shared_ptr<RoutePlanner> route_planner_;

    bool map_ready_ = false;
    bool data_link_ready_ = false;

    bool init_state_set_ = false;
    bool goal_state_set_ = false;
    bool mission_validated_ = false;

    // parameters for route planner
    SimplePoint position_s_;
    SimplePoint position_g_;

    // parameters for motion planner
    CurvilinearGrid::GridCurvePoint pose_s_;
    CurvilinearGrid::GridCurvePoint pose_g_;

    ReferenceRoute latest_requested_route_;

    void ResetMission();
    void ValidateMissionRequest();
};
} // namespace autodrive

#endif /* CAV_MISSION_CONTROL_HPP */
