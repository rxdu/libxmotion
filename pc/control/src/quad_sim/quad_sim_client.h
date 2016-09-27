/*
 * quad_sim_client.h
 *
 *  Created on: Sep 1, 2016
 *      Author: rdu
 */

#ifndef CONTROL_SRC_QUAD_SIM_QUAD_SIM_CLIENT_H_
#define CONTROL_SRC_QUAD_SIM_QUAD_SIM_CLIENT_H_

#include <cstdint>

extern "C" {
    #include "extApi.h"
/*	#include "extApiCustom.h" // custom remote API functions */
}

#include "common/control_types.h"
#include "quad_sim/quad_sim_data.h"
#include "vrep_sim/vrep_interface/robot_sim_client.h"

namespace srcl_ctrl {

class QuadSimClient : public RobotSimClient<QuadDataFromSim, QuadDataToSim>
{
public:
	QuadSimClient();
	QuadSimClient(simxInt clientId);
	~QuadSimClient();

private:
	virtual void ConfigDataStreaming(void);

public:
	virtual bool ReceiveDataFromRobot(QuadDataFromSim& rstate);
	virtual void SendDataToRobot(const QuadDataToSim& rcmd);

private:
	bool ReceiveGyroData(IMU_DataType& data);
	bool ReceiveAccData(IMU_DataType& data);
	bool GetVisionImage(simxUChar img[IMG_RES_Y][IMG_RES_X]);
	bool Get3DScanPoints(std::vector<Point3f>& points);

	bool ReceiveQuadPosition(Point3f& data);
	bool ReceiveQuadVelocity(Point3f& data);
	bool ReceiveQuadOrientation(Point3f& data);
	bool ReceiveQuadQuaternion(Quaternion& data);

	void SendPropellerCmd(QuadCmd cmd);

private:
	const uint64_t max_motor_speed_;

private:
	simxInt quad_handle_;
	simxInt ref_handle_;

private:
	// quadrotor kinematics/dynamics
	IMUData imu_data;
	simxFloat quad_pos[3];
	simxFloat quad_linear_vel[3];
	simxFloat quad_angular_vel[3];
	simxFloat quad_ori[3];
	simxUChar* gyro_sig;
	simxInt gyro_sig_size;
	simxUChar* acc_sig;
	simxInt acc_sig_size;
	simxUChar* quat_sig;
	simxInt quat_sig_size;
	simxUChar* scannerptr_sig;
	simxInt scannerptr_sig_size;

	// vision
	simxInt camera_handle_;
	simxUChar *image_raw_;
	int img_res[2];
};

}

#endif /* CONTROL_SRC_QUAD_SIM_QUAD_SIM_CLIENT_H_ */
