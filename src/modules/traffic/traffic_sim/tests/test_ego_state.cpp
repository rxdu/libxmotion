#include <memory>
#include <iostream>

#include "traffic_map/map_loader.hpp"

#include "cav_common/cav_datalink.hpp"
#include "cav_common/vehicle_state.hpp"

using namespace ivnav;

int main()
{
    std::shared_ptr<LCMLink> data_link_ = std::make_shared<LCMLink>();
    if (!data_link_->good())
        std::cerr << "ERROR: Failed to initialize LCM." << std::endl;

    // generate ego vehicle state
    MapLoader loader("/home/rdu/Workspace/librav/data/road_map/urban_single_lane_loop_full.osm");

    auto cpoint = loader.traffic_map->FindTrafficChannel("s1", "s2")->ConvertToCurvePoint({150, 0});

    std::cout << "distance: " << 150 << " position: " << cpoint.x << " , " << cpoint.y << std::endl;

    VehicleState state(-1, {cpoint.x, cpoint.y, cpoint.theta}, 0);

    librav_lcm_msgs::VehicleState state_msg;

    auto pose = state.GetPose();
    auto fp = state.GetFootprint();

    state_msg.id = state.id_;
    state_msg.position[0] = pose.position.x;
    state_msg.position[1] = pose.position.y;
    state_msg.theta = pose.theta;
    state_msg.speed = state.GetSpeed();

    for (int i = 0; i < 4; ++i)
    {
        auto pt = fp.GetPoint(i);
        state_msg.footprint.points[i].x = pt.x;
        state_msg.footprint.points[i].y = pt.y;
    }

    data_link_->publish(CAV_COMMON_CHANNELS::EGO_VEHICLE_STATE_CHANNEL, &state_msg);

    return 0;
}