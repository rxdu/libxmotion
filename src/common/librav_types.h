/* 
 * librav_types.h
 * 
 * Created on: Nov 06, 2017 11:33
 * Description: common type definitions for librav, evolved from 
 *              librav_types.h and librav_types.h   
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef LIBRAV_TYPES_H
#define LIBRAV_TYPES_H

#include <cstdint>
#include <vector>
#include <iostream>

namespace librav
{

/************ Types for Control ************/
template <typename T>
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
} EulerAngle;

typedef struct
{
    float x;
    float y;
    float z;
    float w;
} Quaternion;

typedef struct
{
    Point3f pos;
    EulerAngle ori;
} Pose;

typedef struct
{
    IMU_DataType gyro;
    IMU_DataType acc;
} IMUData;

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

/************ Types for Planning ************/
template<typename T>
struct position2d
{
	position2d():x(0),y(0){}
	position2d(T _x, T _y):x(_x),y(_y){}

	T x;
	T y;

	bool operator==(const struct position2d& other) const
	{
		if(this->x == other.x && this->y == other.y)
			return true;
		else
			return false;
	}

	friend std::ostream& operator<<(std::ostream& os, const struct position2d& pos)
	{
		os << pos.x << " , " << pos.y;
		return os;
	}
};

template<typename T>
struct position3d
{
	position3d():x(0),y(0),z(0){}
	position3d(T _x, T _y, T _z):x(_x),y(_y),z(_z){}

	T x;
	T y;
	T z;

	bool operator==(const struct position3d& other) const
	{
		if(this->x == other.x && this->y == other.y && this->z == other.z)
			return true;
		else
			return false;
	}

	friend std::ostream& operator<<(std::ostream& os, const struct position3d& pos)
	{
		os << pos.x << " , " << pos.y << " , " << pos.z;
		return os;
	}
};

using Position2D = position2d<int32_t>;
using Position2Dd = position2d<double>;
using Position3D = position3d<int32_t>;
using Position3Dd = position3d<double>;

enum class OccupancyType
{
	FREE,
	OCCUPIED,
	// only above two are used for a graph
	MIXED,
	INTERESTED,
	UNKONWN,
	EXPANDED_OBS
};

typedef struct
{
	uint32_t min;
	uint32_t max;
}Range2D;

typedef struct
{
	Range2D x;
	Range2D y;
}BoundingBox;

typedef struct {
	float positions[3];
	float velocity[3];
	float yaw;

	bool pos_constr;
	bool vel_constr;
	bool yaw_constr;
} Keyframe;

typedef struct {
	std::vector<Keyframe> keyframes;
	uint64_t start_time;
} KeyframeSet;

}

#endif /* LIBRAV_TYPES_H */
