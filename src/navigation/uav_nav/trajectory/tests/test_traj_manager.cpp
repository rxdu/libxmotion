// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

#include "trajectory/trajectory_manager.hpp"

using namespace librav;

UAVTrajectory GenerateTestTrajectory()
{
    UAVTrajectory test_traj;

    int time_stamp1 = 50;
    int time_stamp2 = time_stamp1 + 150;
    int final_time_stamp = time_stamp2 + 10;

    for (int i = 0; i < final_time_stamp; i++)
    {
        UAVTrajectoryPoint pt;
        pt.point_empty = false;

        double height = 0.5;
        double radius = 1.0;
        double circle_ang_vel = 180.0 / 180.0 * 3.14;

        if (i < time_stamp1)
        {
            pt.positions[0] = 0;
            pt.positions[1] = 0;
            pt.positions[2] = 0.5;
            pt.velocities[0] = 0;
            pt.velocities[1] = 0;
            pt.velocities[2] = 0;
            pt.accelerations[0] = 0;
            pt.accelerations[1] = 0;
            pt.accelerations[2] = 0;
            pt.yaw = 0;
            pt.duration = 1;
        }
        else if (i < time_stamp2)
        {
            double angle = (i - time_stamp1) * 0.01 * circle_ang_vel;
            pt.positions[0] = radius * cos(angle - M_PI / 2);
            pt.positions[1] = radius * sin(angle - M_PI / 2);
            pt.positions[2] = height;

            pt.velocities[0] = -radius * sin(angle - M_PI / 2) * 0.01 * circle_ang_vel;
            pt.velocities[1] = radius * cos(angle - M_PI / 2) * 0.01 * circle_ang_vel;
            pt.velocities[2] = 0;

            pt.accelerations[0] = -radius * cos(angle - M_PI / 2) * 0.01 * circle_ang_vel * 0.01 * circle_ang_vel;
            pt.accelerations[1] = -radius * sin(angle - M_PI / 2) * 0.01 * circle_ang_vel * 0.01 * circle_ang_vel;
            pt.accelerations[2] = 0;

            pt.yaw = angle;
            pt.duration = 5;
        }
        else
            pt.point_empty = true;

        test_traj.push_back(pt);
    }

    return test_traj;
}

int main()
{
    std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

	if(!lcm->good())
	{
		std::cout << "ERROR: Failed to initialize LCM." << std::endl;
		return -1;
	}

    TrajectoryManager traj_man(lcm);

    while(true)
    {
        lcm->handleTimeout(0);
    }

    return 0;
}