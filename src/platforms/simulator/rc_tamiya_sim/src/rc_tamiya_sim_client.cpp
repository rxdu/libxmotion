/* 
 * rc_tamiya_sim_client.cpp
 * 
 * Created on: Aug 10, 2017
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "rc_tamiya_sim/rc_tamiya_sim_client.hpp"

#include <iostream>
#include <cmath>

using namespace librav;

RCTamiyaSimClient::RCTamiyaSimClient() : VrepSimClient<DataFromRCTamiyaSim, DataToRCTamiyaSim>()
{
    SetupSim();
}

RCTamiyaSimClient::RCTamiyaSimClient(std::shared_ptr<LCMLink> lcm) : VrepSimClient<DataFromRCTamiyaSim, DataToRCTamiyaSim>(),
                                                                     lcm_(lcm),
                                                                     coordinator_(lcm)
{
    SetupSim();
}

RCTamiyaSimClient::~RCTamiyaSimClient()
{
    delete[] image_raw_;
}

void RCTamiyaSimClient::SetupSim()
{
    // initialize variables
    image_raw_ = new simxUChar[TamiyaTA07ProSimParams::main_cam_res_y * TamiyaTA07ProSimParams::main_cam_res_x];

    img_res[0] = TamiyaTA07ProSimParams::main_cam_res_x;
    img_res[1] = TamiyaTA07ProSimParams::main_cam_res_y;

    // initialize communication between server and client
    ConfigDataStreaming();

    std::cout << "INFO: RC car simulation client initialized successfully." << std::endl;
}

void RCTamiyaSimClient::ConfigDataStreaming(void)
{
    // get simulation object handles
    simxGetObjectHandle(client_id_, "tamiya", &car_handle_, simx_opmode_oneshot_wait);
    simxGetObjectHandle(client_id_, "sensor_ref", &sensor_ref_handle_, simx_opmode_oneshot_wait);

    simxGetObjectHandle(client_id_, "realsense_rgb", &camera_handle_, simx_opmode_oneshot_wait);

    simxGetObjectHandle(client_id_, "steering_joint_fr", &steering_right_, simx_opmode_oneshot_wait);
    simxGetObjectHandle(client_id_, "steering_joint_fl", &steering_left_, simx_opmode_oneshot_wait);

    simxGetObjectHandle(client_id_, "driving_joint_front_right", &driving_front_right_, simx_opmode_oneshot_wait);
    simxGetObjectHandle(client_id_, "driving_joint_front_left", &driving_front_left_, simx_opmode_oneshot_wait);
    simxGetObjectHandle(client_id_, "driving_joint_rear_right", &driving_rear_right_, simx_opmode_oneshot_wait);
    simxGetObjectHandle(client_id_, "driving_joint_rear_left", &driving_rear_left_, simx_opmode_oneshot_wait);

    // initialize robot status data streaming
    simxGetObjectFloatParameter(client_id_, driving_rear_right_, 2012, &driving_right_vel_, simx_opmode_streaming);
    simxGetObjectFloatParameter(client_id_, driving_rear_left_, 2012, &driving_left_vel_, simx_opmode_streaming);

    simxGetVisionSensorImage(client_id_, camera_handle_, img_res, &image_raw_, 1, simx_opmode_streaming);

    simxGetObjectVelocity(client_id_, sensor_ref_handle_, body_lin_vel_, body_ang_vel_, simx_opmode_streaming);
    simxGetJointPosition(client_id_, steering_right_, &steering_angle_, simx_opmode_streaming);

    simxGetObjectPosition(client_id_, sensor_ref_handle_, -1, body_position_, simx_opmode_streaming);
    simxGetObjectOrientation(client_id_, sensor_ref_handle_, -1, body_orientation_, simx_opmode_streaming);
}

bool RCTamiyaSimClient::GetCarDrivingSpeed(float &rvel, float &lvel, float &body_speed)
{
    // get joint position and joint velocity
    if (simxGetObjectVelocity(client_id_, sensor_ref_handle_, body_lin_vel_, body_ang_vel_, simx_opmode_buffer) == simx_error_noerror &&
        simxGetObjectFloatParameter(client_id_, driving_rear_right_, 2012, &driving_right_vel_, simx_opmode_buffer) == simx_error_noerror &&
        simxGetObjectFloatParameter(client_id_, driving_rear_left_, 2012, &driving_left_vel_, simx_opmode_buffer) == simx_error_noerror)
    {
        // here you have a valid value!
        body_speed = std::sqrt(body_lin_vel_[0] * body_lin_vel_[0] + body_lin_vel_[1] * body_lin_vel_[1]);
        rvel = driving_rear_right_;
        lvel = driving_rear_left_;

        return true;
    }
    else
        return false;
}

bool RCTamiyaSimClient::GetCarSteeringAngle(float &data)
{
    // get joint position and joint velocity
    if (simxGetJointPosition(client_id_, steering_right_, &steering_angle_, simx_opmode_buffer) == simx_error_noerror)
    {
        // here you have a valid value!
        //		std::cout << "linear: " << "( "<< body_lin_vel_[0] << "," << body_lin_vel_[1] <<","<<body_lin_vel_[2] << " ) ";
        //		std::cout << "angular: " << "( "<< body_ang_vel_[0] << "," << body_ang_vel_[1] <<","<<body_ang_vel_[2] << " )" << std::endl;

        data = steering_angle_;

        return true;
    }
    else
        return false;
}

bool RCTamiyaSimClient::GetCarPose(float &x, float &y, float &theta)
{
    // get joint position and joint velocity
    if (simxGetObjectPosition(client_id_, sensor_ref_handle_, -1, body_position_, simx_opmode_buffer) == simx_error_noerror &&
        simxGetObjectOrientation(client_id_, sensor_ref_handle_, -1, body_orientation_, simx_opmode_buffer) == simx_error_noerror)
    {
        // here you have a valid value!
        x = body_position_[0];
        y = body_position_[1];

        theta = body_orientation_[2];

        return true;
    }
    else
        return false;
}

bool RCTamiyaSimClient::GetVisionImage(simxUChar img[RS_RGB_IMG_RES_Y][RS_RGB_IMG_RES_X])
{
    if (simxGetVisionSensorImage(client_id_, camera_handle_, img_res, &image_raw_, 1, simx_opmode_buffer) == simx_error_noerror)
    {
        // here you have a valid value!
        int i, j;

        for (i = 0; i < RS_RGB_IMG_RES_Y; i++)
            for (j = 0; j < RS_RGB_IMG_RES_X; j++)
                img[i][j] = image_raw_[(RS_RGB_IMG_RES_Y - i - 1) * RS_RGB_IMG_RES_X + j];

        return true;
    }
    else
        return false;
}

void RCTamiyaSimClient::SetCarDrivingSpeed(float cmd)
{
    // add limits to speed
    if (cmd > TamiyaTA07ProSimParams::max_forward_speed)
        cmd = TamiyaTA07ProSimParams::max_forward_speed;
    if (cmd < TamiyaTA07ProSimParams::max_reverse_speed)
        cmd = TamiyaTA07ProSimParams::max_reverse_speed;

    // front wheel driving
    simxSetJointTargetVelocity(client_id_, driving_front_right_, cmd, simx_opmode_oneshot);
    simxSetJointTargetVelocity(client_id_, driving_front_left_, cmd, simx_opmode_oneshot);

    // rear wheel driving
    simxSetJointTargetVelocity(client_id_, driving_rear_right_, -cmd, simx_opmode_oneshot);
    simxSetJointTargetVelocity(client_id_, driving_rear_left_, -cmd, simx_opmode_oneshot);
}

void RCTamiyaSimClient::SetCarSteeringAngle(float cmd)
{
    double cmd_radian = cmd / 180.0 * M_PI;

    if (cmd_radian > TamiyaTA07ProSimParams::max_steer_angle)
        cmd_radian = TamiyaTA07ProSimParams::max_steer_angle;
    if (cmd_radian < -TamiyaTA07ProSimParams::max_steer_angle)
        cmd_radian = -TamiyaTA07ProSimParams::max_steer_angle;

    simxSetJointTargetPosition(client_id_, steering_right_, cmd_radian, simx_opmode_oneshot);
    simxSetJointTargetPosition(client_id_, steering_left_, cmd_radian, simx_opmode_oneshot);
}

void RCTamiyaSimClient::SetCarSteeringVelocity(float cmd)
{
    simxSetJointTargetVelocity(client_id_, steering_right_, cmd, simx_opmode_oneshot);
    simxSetJointTargetVelocity(client_id_, steering_left_, cmd, simx_opmode_oneshot);
}

bool RCTamiyaSimClient::ReceiveDataFromSimRobot(DataFromRCTamiyaSim *rdata)
{
    if (GetCarDrivingSpeed(rdata->speed_rear_right, rdata->speed_rear_left, rdata->body_speed) &&
        GetCarPose(rdata->body_pose.position.x, rdata->body_pose.position.y, rdata->body_pose.theta) &&
        GetCarSteeringAngle(rdata->steering_angle) &&
        GetVisionImage(rdata->mono_image))
        return true;
    else
        return false;
}

void RCTamiyaSimClient::UpdateCtrlLoop(const DataFromRCTamiyaSim &rdata, DataToRCTamiyaSim *rcmd)
{
    // do processing here or use the coordinator_
    // rcmd->driving_cmd = 50;
    // rcmd->steering_cmd = -5;

    // hand everything to coordinator_, if lcm_ is initialized
    if (lcm_ != nullptr)
    {
        coordinator_.ExchangeData(rdata, rcmd);

        std::cout << "command to sim: " << rcmd->driving_cmd << " , " << rcmd->steering_cmd << std::endl;

        // handle LCM callbacks
        lcm_->handleTimeout(0);
    }
}

void RCTamiyaSimClient::SendDataToSimRobot(const DataToRCTamiyaSim &rcmd)
{
    SetCarDrivingSpeed(rcmd.driving_cmd);
    SetCarSteeringAngle(rcmd.steering_cmd);
}
