#include <iostream>
#include <cstdint>
#include <cmath>

#include "mission/mission_control.hpp"

using namespace autodrive;

int main()
{
    CAVMissionControl mctrl("/home/rdu/Workspace/librav/data/road_map/urban_single_lane_loop_full.osm");

    if (!mctrl.IsReady())
    {
        std::cerr << "ERROR: mission control is not set up properly" << std::endl;
        return -1;
    }

    mctrl.GenerateMissionInitialState("s1", "s2", 180, 0);
    mctrl.GenerateMissionGoalState("s1", "s8", 250, 0);

    mctrl.Run();

    return 0;
}