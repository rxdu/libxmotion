/*
 * virtual_quadrotor.cpp
 *
 *  Created on: Sep 12, 2017
 *      Author: rdu
 */

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "quadrotor/path_repair/sim/virtual_quadrotor.h"

using namespace librav;

VirtualQuadrotor::VirtualQuadrotor(std::shared_ptr<lcm::LCM> lcm) : lcm_(lcm),
                                                                    dsensor_(std::make_shared<SimDepthSensor>()),
                                                                    qplanner_(std::make_shared<SimPathRepair>(lcm, dsensor_)),
                                                                    current_heading_(0),
                                                                    traveled_distance_(0),
                                                                    init_path_found_(false),
                                                                    init_repair_path_cost_(0),
                                                                    run_flag_(1),
                                                                    sim_index_(0),
                                                                    logger_(new CsvLogger("prsim", "/home/rdu/Workspace/librav/data/log/quad/prsim"))
{
}

void VirtualQuadrotor::SetConfig(int32_t map_x, int32_t map_y, int32_t map_z, int32_t height, int32_t sensor_rng)
{
    // set sim map size
    qplanner_->SetMapSize(map_x, map_y, map_z);

    // set initial and goal pose
    init_pos_ = Position2D(0, 0);
    init_height_ = height;

    qplanner_->SetStartPosition(init_pos_);
    qplanner_->SetStartHeight(init_height_);

    qplanner_->SetGoalPosition(Position2D(map_x - 1, map_y - 1));
    qplanner_->SetGoalHeight(height);

    qplanner_->SetSensorRange(sensor_rng);

    current_pos_ = init_pos_;
    current_height_ = init_height_;
}

void VirtualQuadrotor::SetSensorRange(int32_t rng)
{
    qplanner_->SetSensorRange(rng);
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

    qplanner_->SetSensorRange(2);

    current_pos_ = init_pos_;
    current_height_ = init_height_;
}

void VirtualQuadrotor::Load_10by10_Config()
{
    // set sim map size
    qplanner_->SetMapSize(10, 10, 5);

    // set initial and goal pose
    init_pos_ = Position2D(0, 0);
    init_height_ = 2;

    qplanner_->SetStartPosition(init_pos_);
    qplanner_->SetStartHeight(init_height_);

    qplanner_->SetGoalPosition(Position2D(9, 9));
    qplanner_->SetGoalHeight(2);

    qplanner_->SetSensorRange(5);

    current_pos_ = init_pos_;
    current_height_ = init_height_;
}

void VirtualQuadrotor::Load_15by20_Config()
{
    // set sim map size
    qplanner_->SetMapSize(15, 20, 5);

    // set initial and goal pose
    init_pos_ = Position2D(0, 0);
    init_height_ = 2;

    qplanner_->SetStartPosition(init_pos_);
    qplanner_->SetStartHeight(init_height_);

    qplanner_->SetGoalPosition(Position2D(14, 19));
    qplanner_->SetGoalHeight(2);

    // qplanner_->SetSensorRange(8);
    qplanner_->SetSensorRange(5);

    current_pos_ = init_pos_;
    current_height_ = init_height_;
}

void VirtualQuadrotor::Load_20by30_Config()
{
    // set sim map size
    qplanner_->SetMapSize(20, 30, 5);

    // set initial and goal pose
    init_pos_ = Position2D(0, 0);
    init_height_ = 2;

    qplanner_->SetStartPosition(init_pos_);
    qplanner_->SetStartHeight(init_height_);

    qplanner_->SetGoalPosition(Position2D(19, 29));
    qplanner_->SetGoalHeight(2);

    qplanner_->SetSensorRange(10);

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

    qplanner_->SetSensorRange(10);

    current_pos_ = init_pos_;
    current_height_ = init_height_;
}

void VirtualQuadrotor::Load_50by50_Config()
{
    // set sim map size
    qplanner_->SetMapSize(50, 50, 5);

    // set initial and goal pose
    init_pos_ = Position2D(0, 0);
    init_height_ = 3;

    qplanner_->SetStartPosition(init_pos_);
    qplanner_->SetStartHeight(init_height_);

    qplanner_->SetGoalPosition(Position2D(49, 49));
    qplanner_->SetGoalHeight(3);

    qplanner_->SetSensorRange(10);

    current_pos_ = init_pos_;
    current_height_ = init_height_;
}

void VirtualQuadrotor::Load_45by60_Config()
{
    // set sim map size
    qplanner_->SetMapSize(45, 60, 5);

    // set initial and goal pose
    init_pos_ = Position2D(0, 0);
    init_height_ = 3;

    qplanner_->SetStartPosition(init_pos_);
    qplanner_->SetStartHeight(init_height_);

    qplanner_->SetGoalPosition(Position2D(44, 59));
    qplanner_->SetGoalHeight(3);

    qplanner_->SetSensorRange(12);

    current_pos_ = init_pos_;
    current_height_ = init_height_;
}

bool VirtualQuadrotor::IsReady()
{
    return qplanner_->IsConfigComplete();
}

double VirtualQuadrotor::CalcWaypointDistance(Position2D pos1, Position2D pos2)
{
    double x1, x2, y1, y2;

    x1 = pos1.x;
    y1 = pos1.y;

    x2 = pos2.x;
    y2 = pos2.y;

    // static_cast: can get wrong result to use "unsigned long" type for deduction
    long x_error = static_cast<long>(x1) - static_cast<long>(x2);
    long y_error = static_cast<long>(y1) - static_cast<long>(y2);

    double cost = std::sqrt(x_error * x_error + y_error * y_error);

    return cost;
}

void VirtualQuadrotor::MoveForward(bool enable_path_repair)
{
    // if active_path_ is not empty, then set the next waypoint to be current pose
    if (active_path_.size() >= 2)
    {
        // find next waypoint
        Position2D next_pt;
        int32_t next_idx = 0;

        for (int i = 0; i < active_path_.size(); i++)
        {
            if (active_path_[i].x != current_pos_.x ||
                active_path_[i].y != current_pos_.y ||
                active_path_[i].z != current_height_)
            {
                next_idx = i;
                break;
            }
        }

        Position2D pos1(current_pos_.x, current_pos_.y);
        Position2D pos2(active_path_[next_idx].x, active_path_[next_idx].y);
        traveled_distance_ += CalcWaypointDistance(pos1, pos2);

        // update current pose
        current_pos_.x = active_path_[next_idx].x;
        current_pos_.y = active_path_[next_idx].y;
        current_height_ = active_path_[next_idx].z;

        if (enable_path_repair)
        {
            current_heading_ = active_path_[next_idx].yaw;
        }
        else
        {
            if(active_path_.size() == 2)
            {
                current_heading_ = 0;
            }
            else
            {
                current_heading_ = atan2(active_path_[next_idx+1].y - active_path_[next_idx].y, 
                    active_path_[next_idx+1].x - active_path_[next_idx].x);
            }
        }

        active_path_.erase(active_path_.begin());
    }
}

void VirtualQuadrotor::PublishState()
{
    //std::cout << "** traveled distance: " << traveled_distance_ << std::endl;

    srcl_lcm_msgs::QuadrotorTransform trans_msg;
    srcl_lcm_msgs::Pose_t trans_base2world;
    srcl_lcm_msgs::Pose_t trans_laser2base;

    Eigen::Quaterniond quat(Eigen::AngleAxisd(current_heading_, Eigen::Vector3d::UnitZ()));
    trans_base2world.position[0] = current_pos_.x + 0.5;
    trans_base2world.position[1] = current_pos_.y + 0.5;
    trans_base2world.position[2] = current_height_ + 0.5;

    trans_base2world.quaternion[0] = quat.w();
    trans_base2world.quaternion[1] = quat.x();
    trans_base2world.quaternion[2] = quat.y();
    trans_base2world.quaternion[3] = quat.z();

    trans_laser2base.position[0] = 0.0;
    trans_laser2base.position[1] = 0.0;
    trans_laser2base.position[2] = 0.0;

    trans_laser2base.quaternion[0] = 1.0;
    trans_laser2base.quaternion[1] = 0;
    trans_laser2base.quaternion[2] = 0;
    trans_laser2base.quaternion[3] = 0;

    trans_msg.base_to_world = trans_base2world;
    trans_msg.laser_to_base = trans_laser2base;

    lcm_->publish("quad_data/quad_transform", &trans_msg);
}

void VirtualQuadrotor::Step()
{
    if (qplanner_->map_received_)
    {
        // update quadrotor state
        MoveForward();

        // update planner
        auto new_path = qplanner_->UpdatePath(current_pos_, current_height_, current_heading_);

        if (new_path.size() > 1)
        {
            active_path_ = new_path;

            if (!init_path_found_)
            {
                init_path_found_ = true;

                for (auto it = active_path_.begin(); it != active_path_.end() - 1; it++)
                {
                    Position2D pos1((*it).x, (*it).y);
                    Position2D pos2((*(it + 1)).x, (*(it + 1)).y);
                    init_repair_path_cost_ += CalcWaypointDistance(pos1, pos2);
                }
            }
        }

        PublishState();

        if (active_path_.size() == 1)
        {
            // calculate shortcut distance
            double shortest_path = qplanner_->GetGlobal2DPathCost();
            double shortend_dist = shortest_path - traveled_distance_;
            std::cout << "** shorted path :" << shortest_path << " , init repair path: " << init_repair_path_cost_ << std::endl;
            std::cout << "** path shortened by :" << shortend_dist << std::endl;

            // log data for analysis
            logger_->LogData(sim_index_, shortest_path, init_repair_path_cost_, shortend_dist, shortend_dist / shortest_path);

            // reset quadrotor state
            current_pos_ = init_pos_;
            current_height_ = init_height_;
            traveled_distance_ = 0.0;
            init_path_found_ = false;
            init_repair_path_cost_ = 0.0;

            // reset planner
            qplanner_->ResetPlanner();
            ++sim_index_;
        }
    }
    else
    {
        qplanner_->RequestNewMap();
    }
}

void VirtualQuadrotor::CmpStep()
{
    if (qplanner_->map_received_ && run_flag_ != 0)
    {
        // update quadrotor state and planner
        SimPath new_path;
        if (run_flag_ == 1)
        {
            MoveForward();
            new_path = qplanner_->UpdatePath(current_pos_, current_height_, current_heading_);
        }
        else
        {
            MoveForward(false);
            new_path = qplanner_->UpdatePath(current_pos_, current_height_, current_heading_, false);
        }

        if (new_path.size() > 1)
        {
            active_path_ = new_path;

            if (!init_path_found_)
            {
                init_path_found_ = true;

                for (auto it = active_path_.begin(); it != active_path_.end() - 1; it++)
                {
                    Position2D pos1((*it).x, (*it).y);
                    Position2D pos2((*(it + 1)).x, (*(it + 1)).y);
                    init_repair_path_cost_ += CalcWaypointDistance(pos1, pos2);
                }
            }
        }

        PublishState();

        if (active_path_.size() == 1)
        {
            // calculate shortcut distance
            double shortest_path = qplanner_->GetGlobal2DPathCost();
            double shortend_dist = shortest_path - traveled_distance_;
            std::cout << "** shorted path :" << shortest_path << " , init repair path: " << init_repair_path_cost_ << std::endl;
            std::cout << "** path shortened by :" << shortend_dist << std::endl;

            // log data for analysis
            logger_->LogData(sim_index_, run_flag_, shortest_path, init_repair_path_cost_, shortend_dist, shortend_dist / shortest_path);

            // reset quadrotor state
            current_pos_ = init_pos_;
            current_height_ = init_height_;
            traveled_distance_ = 0.0;
            init_path_found_ = false;
            init_repair_path_cost_ = 0.0;

            // update run flag
            switch (run_flag_)
            {
            case 1:
                run_flag_ = 2;
                std::cout << "*********************** finished path repair run ***********************" << std::endl;
                break;
            case 2:
                run_flag_ = 0;
                std::cout << "*********************** finished shortest path run ***********************" << std::endl;
                break;
            }

            // reset planner
            qplanner_->ResetPlanner();      
            
            if(run_flag_ != 0)
                qplanner_->map_received_ = true;
            else
                ++sim_index_;
        }
    }
    else
    {
        qplanner_->RequestNewMap();
        run_flag_ = 1;
    }
}