/* 
 * rc_tamiya_sim_client.hpp
 * 
 * Created on: Aug 10, 2017
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RC_TAMIYA_SIM_CLIENT_HPP
#define RC_TAMIYA_SIM_CLIENT_HPP

#include <cstdint>

#include "common/librav_types.hpp"
#include "datalink/lcm_link.hpp"

#include "rc_tamiya_sim/rc_tamiya_sim_types.hpp"
#include "rc_tamiya_sim/rc_tamiya_sim_coordinator.hpp"
#include "vrep_interface/vrep_sim_client.hpp"

namespace librav
{
class RCTamiyaSimClient : public VrepSimClient<DataFromRCTamiyaSim, DataToRCTamiyaSim>
{
  public:
    RCTamiyaSimClient();
    RCTamiyaSimClient(std::shared_ptr<LCMLink> lcm);

    ~RCTamiyaSimClient();

    virtual bool ReceiveDataFromSimRobot(DataFromRCTamiyaSim *rdata) override;
    virtual void UpdateCtrlLoop(const DataFromRCTamiyaSim &rdata, DataToRCTamiyaSim *rcmd) override;
    virtual void SendDataToSimRobot(const DataToRCTamiyaSim &rcmd) override;

  private:
    std::shared_ptr<LCMLink> lcm_;
    RCTamiyaSimCoordinator coordinator_;

    // sim variables
    simxInt car_handle_;
    simxInt sensor_ref_handle_;
    simxInt camera_handle_;
    simxInt steering_right_;
    simxInt steering_left_;
    simxInt driving_front_right_;
    simxInt driving_front_left_;
    simxInt driving_rear_right_;
    simxInt driving_rear_left_;

    float body_lin_vel_[3];
    float body_ang_vel_[3];
    float body_vel_;

    float body_position_[3];
    float body_orientation_[3];

    float driving_right_vel_;
    float driving_left_vel_;
    float steering_angle_;
    float steering_vel_;

    simxUChar *image_raw_;
    int img_res[2];

    void SetupSim();
    virtual void ConfigDataStreaming(void) override;

    bool GetCarDrivingSpeed(float &rvel, float &lvel, float &body_speed);
    bool GetCarSteeringAngle(float &data);
    bool GetCarPose(float &x, float &y, float &theta);

    bool GetVisionImage(simxUChar img[RS_RGB_IMG_RES_Y][RS_RGB_IMG_RES_X]);

    void SetCarSteeringAngle(float cmd);
    void SetCarSteeringVelocity(float cmd);
    void SetCarDrivingSpeed(float cmd);
};
} // namespace librav

#endif /* RC_TAMIYA_SIM_CLIENT_HPP */
