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

// time_stamp starts from 0 when system initialized, increases at step 1 ms
typedef uint64_t time_stamp;

/****************** Types for Control ******************/
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

struct EulerAngle
{
	float roll;
	float pitch;
	float yaw;
};

struct Quaternion
{
	float x;
	float y;
	float z;
	float w;
};

struct Pose
{
	Point3f pos;
	EulerAngle ori;
};

struct IMUData
{
	Point3f gyro;
	Point3f acc;
};

struct UAVTrajectoryPoint
{
	bool point_empty;
	float positions[3];
	float velocities[3];
	float accelerations[3];
	float jerks[3];
	float yaw;
	float yaw_rate;
	uint64_t duration; // in milliseconds
};

typedef std::vector<UAVTrajectoryPoint> UAVTrajectory;

/****************** Types for Planning ******************/
template <typename T>
struct position2d
{
	position2d() : x(0), y(0) {}
	position2d(T _x, T _y) : x(_x), y(_y) {}

	T x;
	T y;

	bool operator==(const struct position2d &other) const
	{
		if (this->x == other.x && this->y == other.y)
			return true;
		else
			return false;
	}

	friend std::ostream &operator<<(std::ostream &os, const struct position2d &pos)
	{
		os << pos.x << " , " << pos.y;
		return os;
	}
};

template <typename T>
struct position3d
{
	position3d() : x(0), y(0), z(0) {}
	position3d(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}

	T x;
	T y;
	T z;

	bool operator==(const struct position3d &other) const
	{
		if (this->x == other.x && this->y == other.y && this->z == other.z)
			return true;
		else
			return false;
	}

	friend std::ostream &operator<<(std::ostream &os, const struct position3d &pos)
	{
		os << pos.x << " , " << pos.y << " , " << pos.z;
		return os;
	}
};

using Position2Di = position2d<int32_t>;
using Position2Dd = position2d<double>;
using Position3Di = position3d<int32_t>;
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

struct Range2D
{
	uint32_t min;
	uint32_t max;
};

struct BoundingBox
{
	Range2D x;
	Range2D y;
};

struct Keyframe
{
	float position[3];
	float velocity[3];
	float yaw;

	bool pos_constr;
	bool vel_constr;
	bool yaw_constr;
};

struct KeyframeSet
{
	std::vector<Keyframe> keyframes;
	uint64_t start_time;
};

}

#endif /* LIBRAV_TYPES_H */
