/*
 * librav_types.h
 *
 *  Created on: Jul 19, 2015
 *      Author: rdu
 */

#ifndef COMMON_CONTROL_TYPES_H_
#define COMMON_CONTROL_TYPES_H_

#include <cstdint>
#include <vector>

namespace librav{

template<typename T>
struct point3
{
	T x;
	T y;
	T z;
};

using Point3f = point3<float>;
using Point3d = point3<double>;
using Point3i = point3<int32_t>;

typedef Point3f IMU_DataType;

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
	float jerks[3];
	float yaw;
	float yaw_rate;
	uint64_t duration; // in milliseconds
} UAVTrajectoryPoint;

typedef std::vector<UAVTrajectoryPoint> UAVTrajectory;

// time_stamp starts from 0 when system initialized, increases at step 1 ms
typedef uint64_t time_stamp;

}

#endif /* COMMON_CONTROL_TYPES_H_ */
