/* 
 * rc_tamiya_sim_coordinator.hpp
 * 
 * Created on: Oct 12, 2018 11:01
 * Description: simulated version of car nerve system
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RC_TAMIYA_SIM_COORDINATOR_HPP
#define RC_TAMIYA_SIM_COORDINATOR_HPP

#include <memory>

#include "datalink/lcm_link.hpp"
#include "rc_tamiya_sim/rc_tamiya_sim_types.hpp"

namespace librav
{
class RCTamiyaSimCoordinator
{
  public:
    RCTamiyaSimCoordinator() = default;
    RCTamiyaSimCoordinator(std::shared_ptr<LCMLink> lcm);

    float latest_driving_cmd_ = 0.0;
    float latest_steering_cmd_ = 0.0;

    void InitLogger(std::string log_name_prefix, std::string log_save_path);
    void ExchangeData(const DataFromRCTamiyaSim &rdata, DataToRCTamiyaSim *rcmd);

  private:
    std::shared_ptr<LCMLink> lcm_;
    int32_t cmd_timeout_counter_ = 0;

    void HandleLCMMessage_CarCmd(const lcm_link::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::CarCommand_t *msg);

    void PublishRawIMUData(const IMU6DOFData &msg);
    void PublishBodyPose(const Pose2f &msg);
    void PublishRawSpeedData(const Speed &msg);
};
} // namespace librav

#endif /* RC_TAMIYA_SIM_COORDINATOR_HPP */
