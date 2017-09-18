/*
 * virtual_quadrotor.cpp
 *
 *  Created on: Sep 12, 2017
 *      Author: rdu
 */

#include "quadrotor/path_repair/sim/virtual_quadrotor.h"

using namespace librav;

VirtualQuadrotor::VirtualQuadrotor(std::shared_ptr<lcm::LCM> lcm) : lcm_(lcm),
                                                                    qplanner_(std::make_shared<SimPathRepair>(lcm)),
                                                                    current_heading_(0)
{
}

void VirtualQuadrotor::Load_5by5_Config()
{
    // set sim map size
    qplanner_->SetMapSize(5, 5, 5);
    
    // set initial and goal pose
    init_pos_ = Position2D(0, 0);
    init_height_ = 2;

    qplanner_->SetStartPosition(init_pos_);
    qplanner_->SetStartHeight(init_height_);

    qplanner_->SetGoalPosition(Position2D(4, 4));
    qplanner_->SetGoalHeight(2);

    current_pos_ = init_pos_;
    current_height_ = init_height_;
}

void VirtualQuadrotor::Load_10by10_Config()
{
    // set sim map size
    qplanner_->SetMapSize(10, 10, 5);

    // set initial and goal pose
    init_pos_ = Position2D(0, 0);
    init_height_ = 0;

    qplanner_->SetStartPosition(init_pos_);
    qplanner_->SetStartHeight(init_height_);

    qplanner_->SetGoalPosition(Position2D(9, 9));
    qplanner_->SetGoalHeight(0);

    current_pos_ = init_pos_;
    current_height_ = init_height_;
}

void VirtualQuadrotor::Load_30by50_Config()
{
    // set sim map size
    qplanner_->SetMapSize(30, 50, 5);

    // set initial and goal pose
    init_pos_ = Position2D(0, 0);
    init_height_ = 3;

    qplanner_->SetStartPosition(init_pos_);
    qplanner_->SetStartHeight(init_height_);

    qplanner_->SetGoalPosition(Position2D(29, 49));
    qplanner_->SetGoalHeight(3);
}

bool VirtualQuadrotor::IsReady()
{
    return qplanner_->IsConfigComplete();
}

void VirtualQuadrotor::MoveForward()
{
    // active_path_ should always start from current pose
    // if active_path_ is not empty, then set the next waypoint to be current pose
    if (active_path_.size() >= 2)
    {
        current_pos_.x = active_path_[1].x;
        current_pos_.y = active_path_[1].y;
        current_height_ = active_path_[1].z;
        current_heading_ = active_path_[1].yaw;
    }
}

void VirtualQuadrotor::Step()
{
    if (qplanner_->map_received_)
    {
        // update quadrotor state
        MoveForward();

        // update planner
        active_path_ = qplanner_->UpdatePath(current_pos_, current_height_, current_heading_);

        if (active_path_.size() == 1)
        {
            // reset quadrotor state
            current_pos_ = init_pos_;
            current_height_ = init_height_;

            // reset planner
            qplanner_->ResetPlanner();
        }
    }
    else
    {
        qplanner_->RequestNewMap();
    }
}