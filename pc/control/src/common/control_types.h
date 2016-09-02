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

}

#endif /* COMMON_CONTROL_TYPES_H_ */
