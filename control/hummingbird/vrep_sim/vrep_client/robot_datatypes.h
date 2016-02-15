/*
 * quadsim_datatypes.h
 *
 *  Created on: Jul 19, 2015
 *      Author: rdu
 */

#ifndef QUADSIM_CLIENT_QUADSIM_DATATYPES_H_
#define QUADSIM_CLIENT_QUADSIM_DATATYPES_H_

namespace srcl_ctrl{

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
	float ang_vel[4];
}QuadCmd;

#define IMG_RES_X 160
#define IMG_RES_Y 90

typedef struct
{
	unsigned char mono_image[IMG_RES_Y][IMG_RES_X];
	IMUData imu_data;
	Position pos;
} DataFromRobot;

typedef struct
{
	QuadCmd motor_cmd;
} DataToRobot;

}

#endif /* QUADSIM_CLIENT_QUADSIM_DATATYPES_H_ */
