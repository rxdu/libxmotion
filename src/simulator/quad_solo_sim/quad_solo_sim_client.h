/*
 * quad_solo_sim_client.h
 *
 *  Created on: Oct 22, 2016
 *      Author: rdu
 */

#ifndef SIMULATOR_QUAD_SOLO_SIM_CLIENT_H_
#define SIMULATOR_QUAD_SOLO_SIM_CLIENT_H_

#include <cstdint>

extern "C" {
    #include "extApi.h"
/*	#include "extApiCustom.h" // custom remote API functions */
}

#include "common/librav_types.h"

#include "vrep_sim/vrep_interface/robot_sim_client.h"
#include "simulator/quad_solo_sim/quad_solo_sim_types.h"

namespace librav {

class QuadSoloSimClient : public RobotSimClient<DataFromQuadSim, DataToQuadSim>
{
public:
	QuadSoloSimClient();
	QuadSoloSimClient(simxInt clientId);
	~QuadSoloSimClient();

private:
	virtual void ConfigDataStreaming(void);

public:
	virtual bool ReceiveDataFromRobot(DataFromQuadSim& rstate);
	virtual void SendDataToRobot(const DataToQuadSim& rcmd);

private:
	bool ReceiveGyroData(IMU_DataType& data);
	bool ReceiveAccData(IMU_DataType& data);
	bool GetVisionImage(simxUChar img[IMG_RES_Y][IMG_RES_X]);
	bool Get3DScanPoints(std::vector<Point3f>& points);

	bool ReceiveQuadPosition(Point3f& data);
	bool ReceiveQuadVelocity(Point3f& data);
	bool ReceiveQuadOrientation(Point3f& data);
	bool ReceiveQuadQuaternion(Quaternion& data);

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

#endif /* SIMULATOR_QUAD_SOLO_SIM_CLIENT_H_ */
