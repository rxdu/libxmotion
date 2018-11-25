/* 
 * rc_tamiya_sim_coordinator.cpp
 * 
 * Created on: Oct 12, 2018 11:01
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "rc_tamiya_sim/rc_tamiya_sim_coordinator.hpp"

#include <iostream>

#include "logging/logger.hpp"
#include "params/lcm_channels.hpp"

using namespace librav;

RCTamiyaSimCoordinator::RCTamiyaSimCoordinator(std::shared_ptr<LCMLink> lcm) : lcm_(lcm)
{
    lcm_->subscribe(LCM_CHANNELS::CAR_COMMOND_CHANNEL, &RCTamiyaSimCoordinator::HandleLCMMessage_CarCmd, this);
}

void RCTamiyaSimCoordinator::InitLogger(std::string log_name_prefix, std::string log_save_path)
{
    CtrlLogger &logging_helper = CtrlLogger::GetLogger(log_name_prefix, log_save_path);

    logging_helper.AddItemNameToEntryHead("pos_x");
    logging_helper.AddItemNameToEntryHead("pos_y");
    logging_helper.AddItemNameToEntryHead("pos_z");
    logging_helper.AddItemNameToEntryHead("pos_d_x");
    logging_helper.AddItemNameToEntryHead("pos_d_y");
    logging_helper.AddItemNameToEntryHead("pos_d_z");
    logging_helper.AddItemNameToEntryHead("pos_e_x");
    logging_helper.AddItemNameToEntryHead("pos_e_y");
    logging_helper.AddItemNameToEntryHead("pos_e_z");

    logging_helper.PassEntryHeaderToLogger();
}

void RCTamiyaSimCoordinator::ExchangeData(const DataFromRCTamiyaSim &rdata, DataToRCTamiyaSim *rcmd)
{
    // republish data to LCM
    PublishRawIMUData(rdata.imu_data);
    PublishRawSpeedData(Speed(0, rdata.body_speed));
    PublishBodyPose(rdata.body_pose);

    // get latest command
    rcmd->driving_cmd = latest_driving_cmd_;
    rcmd->steering_cmd = latest_steering_cmd_;

    // reset command values if not updated for 5 control iterations (~100ms)
    if (cmd_timeout_counter_++ > 5)
    {
        latest_driving_cmd_ = 0.0;
        latest_steering_cmd_ = 0.0;
    }
}

void RCTamiyaSimCoordinator::HandleLCMMessage_CarCmd(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::CarCommand_t *msg)
{
    std::cout << "received command: " << msg->servo << " , " << msg->motor << std::endl;

    if (msg->update_servo)
        latest_steering_cmd_ = msg->servo * TamiyaTA07ProSimParams::max_steer_angle;

    if (msg->update_motor)
    {
        if (msg->motor >= 0)
            latest_driving_cmd_ = msg->motor * TamiyaTA07ProSimParams::max_forward_speed;
        else
            latest_driving_cmd_ = msg->motor * TamiyaTA07ProSimParams::max_reverse_speed;
    }

    cmd_timeout_counter_ = 0;
}

void RCTamiyaSimCoordinator::PublishRawIMUData(const IMU6DOFData &msg)
{
    librav_lcm_msgs::CarRawIMU_t imu_msg;

    imu_msg.mtime = 0;

    imu_msg.gyro[0] = msg.gyro.x;
    imu_msg.gyro[1] = msg.gyro.y;
    imu_msg.gyro[2] = msg.gyro.z;

    imu_msg.accel[0] = msg.accel.x;
    imu_msg.accel[1] = msg.accel.y;
    imu_msg.accel[2] = msg.accel.z;

    imu_msg.magn[0] = 0;
    imu_msg.magn[1] = 0;
    imu_msg.magn[2] = 0;

    lcm_->publish(LCM_CHANNELS::CAR_RAW_IMU_CHANNEL, &imu_msg);
}

void RCTamiyaSimCoordinator::PublishBodyPose(const Pose2f &msg)
{
    librav_lcm_msgs::CarPose_t pose_msg;

    pose_msg.mtime = 0;
    pose_msg.position.x = msg.position.x;
    pose_msg.position.y = msg.position.y;
    pose_msg.theta = msg.theta;

    lcm_->publish(LCM_CHANNELS::CAR_BODY_POSE_CHANNEL, &pose_msg);
}

void RCTamiyaSimCoordinator::PublishRawSpeedData(const Speed &msg)
{
    librav_lcm_msgs::CarRawSpeed_t spd_msg;

    spd_msg.mtime = msg.mtime;
    spd_msg.speed = msg.speed;

    lcm_->publish(LCM_CHANNELS::CAR_RAW_SPEED_CHANNEL, &spd_msg);
}