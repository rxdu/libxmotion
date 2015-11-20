/*
 * quadsim_datatypes.h
 *
 *  Created on: Jul 19, 2015
 *      Author: rdu
 */

#ifndef QUADSIM_CLIENT_QUADSIM_DATATYPES_H_
#define QUADSIM_CLIENT_QUADSIM_DATATYPES_H_

namespace RobotToolkitRIVeR{

typedef struct
{
	float raw_x;
	float raw_y;
	float raw_z;
}IMU_DataType;

typedef struct
{
	float x;
	float y;
	float z;
}Position;

typedef struct
{
	float roll;
	float pitch;
	float yaw;
}EulerAngle;

typedef struct
{
	Position pos;
	EulerAngle ori;
}Pose;

typedef struct
{
	IMU_DataType gyro;
	IMU_DataType acc;
}IMUData;

typedef struct
{
	float q[2];
	float q_dot[2];
	float q_ddot[2];
}JointState;

typedef struct
{
	float torque[2];
	float velocity[2];
}JointCmd;

typedef struct
{
	float body_vel;
	float driving_vel_right;
	float driving_vel_left;
	float steering_ang;
}VehicleState;

typedef struct
{
	float driving_vel_rcmd;
	float driving_vel_lcmd;
	float steering_ang_cmd;
}VehicleCmd;

#define IMG_RES_X 160
#define IMG_RES_Y 90

typedef struct
{
	VehicleState vs;
	unsigned char mono_image[IMG_RES_Y][IMG_RES_X];
} DataFromRobot;

typedef struct
{
	VehicleCmd cmd;
} DataToRobot;

}

#endif /* QUADSIM_CLIENT_QUADSIM_DATATYPES_H_ */
