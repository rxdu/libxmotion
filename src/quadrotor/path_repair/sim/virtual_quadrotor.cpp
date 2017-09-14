/*
 * virtual_quadrotor.cpp
 *
 *  Created on: Sep 12, 2017
 *      Author: rdu
 */

#include "quadrotor/path_repair/sim/virtual_quadrotor.h"

using namespace librav;

VirtualQuadrotor::VirtualQuadrotor(std::shared_ptr<lcm::LCM> lcm) : lcm_(lcm),
                                                                    reached_goal_(false),
                                                                    qplanner_(std::make_shared<SimPathRepair>(lcm))
{
}

void VirtualQuadrotor::Load_30by50_Config()
{
    qplanner_->SetStartPosition(Position2D(0, 0));
    qplanner_->SetGoalPosition(Position2D(29, 49));
    qplanner_->SetGoalHeight(3);
}

bool VirtualQuadrotor::MoveForward()
{
    return false;
}

void VirtualQuadrotor::Step()
{
    if (qplanner_->map_received_ && !reached_goal_)
    {
        // update quadrotor state
        reached_goal_ = MoveForward();

        // update planner
        qplanner_->UpdatePath();
    }
    else
    {
        qplanner_->RequestNewMap();
    }
}