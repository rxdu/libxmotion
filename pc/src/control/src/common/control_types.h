/*
 * control_types.h
 *
 *  Created on: Jul 19, 2015
 *      Author: rdu
 */

#ifndef COMMON_CONTROL_TYPES_H_
#define COMMON_CONTROL_TYPES_H_

#include <cstdint>
#include <vector>

namespace srcl_ctrl{

/****************************** Control Common ******************************/

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
}Point3f;

typedef struct
{
	float roll;
	float pitch;
	float yaw;
}EulerAngle;

typedef struct
{
	float x;
	float y;
	float z;
	float w;
}Quaternion;

typedef struct
{
	Point3f pos;
	EulerAngle ori;
}Pose;

typedef struct
{
	IMU_DataType gyro;
	IMU_DataType acc;
}IMUData;

typedef struct
{
	bool point_empty;
	float positions[3];
	float velocities[3];
	float accelerations[3];
	float yaw;
	uint64_t duration; // in milliseconds
} UAVTrajectoryPoint;

typedef std::vector<UAVTrajectoryPoint> UAVTrajectory;

/******************************* Quadrotor Types ****************************/
typedef struct
{
	float ang_vel[4];
}QuadCmd;

enum class QuadFlightType {
	X_TYPE,
	PLUS_TYPE,
};

#define IMG_RES_X 160
#define IMG_RES_Y 90

typedef struct
{
	// sensor data
	unsigned char mono_image[IMG_RES_Y][IMG_RES_X];
	IMUData imu_data;

	// data only available in simulator
	Point3f pos_i;
	Point3f vel_i;
	Point3f rot_i;
	Quaternion quat_i;
	Point3f rot_rate_b;
} DataFromQuad;

typedef struct
{
	QuadCmd motor_cmd;
} DataToQuad;

/****************************************************************************/

}

#endif /* COMMON_CONTROL_TYPES_H_ */
