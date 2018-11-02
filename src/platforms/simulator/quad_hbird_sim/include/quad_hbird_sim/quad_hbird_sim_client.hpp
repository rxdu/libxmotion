/* 
 * quad_hbird_sim_client.hpp
 * 
 * Created on: Sep 1, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef QUAD_HBIRD_SIM_CLIENT_HPP
#define QUAD_HBIRD_SIM_CLIENT_HPP

#include <cstdint>

#include "common/librav_types.hpp"
#include "vrep_interface/vrep_sim_client.hpp"
#include "quad_hbird_sim/quad_hbird_sim_types.hpp"
#include "quad_hbird_sim/quad_hbird_sim_control.hpp"

namespace librav
{

class QuadHbirdSimClient : public VrepSimClient<DataFromQuadSim, DataToQuadSim>
{
  public:
	QuadHbirdSimClient(simxInt clientId = 0);
	~QuadHbirdSimClient();

	virtual bool ReceiveDataFromSimRobot(DataFromQuadSim *rdata) override;
	virtual void UpdateCtrlLoop(const DataFromQuadSim &rdata, DataToQuadSim *rcmd) override;
	virtual void SendDataToSimRobot(const DataToQuadSim &rcmd) override;

	QuadHbirdSimControl quad_ctrl;

  private:
	const uint64_t max_motor_speed_ = 10000;

	simxInt quad_handle_;
	simxInt ref_handle_;

	// quadrotor kinematics/dynamics
	IMU6DOFData imu_data;
	simxFloat quad_pos[3];
	simxFloat quad_linear_vel[3];
	simxFloat quad_angular_vel[3];
	simxFloat quad_ori[3];
	simxUChar *gyro_sig;
	simxInt gyro_sig_size;
	simxUChar *acc_sig;
	simxInt acc_sig_size;
	simxUChar *quat_sig;
	simxInt quat_sig_size;
	simxUChar *scannerptr_sig;
	simxInt scannerptr_sig_size;

	// vision
	simxInt camera_handle_;
	simxUChar *image_raw_;
	int img_res[2];

	virtual void ConfigDataStreaming(void) override;

	bool ReceiveGyroData(Point3f &data);
	bool ReceiveAccData(Point3f &data);
	bool GetVisionImage(simxUChar img[IMG_RES_Y][IMG_RES_X]);
	bool Get3DScanPoints(std::vector<Point3f> &points);

	bool ReceiveQuadPosition(Point3f &data);
	bool ReceiveQuadVelocity(Point3f &data);
	bool ReceiveQuadOrientation(Point3f &data);
	bool ReceiveQuadQuaternion(Quaternion &data);
};
}

#endif /* QUAD_HBIRD_SIM_CLIENT_HPP */
