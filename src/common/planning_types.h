/*
 * common_types.h
 *
 *  Created on: Jan 28, 2016
 *      Author: rdu
 */

#ifndef SRC_MAP_COMMON_TYPES_H_
#define SRC_MAP_COMMON_TYPES_H_

#include <cstdint>
#include <vector>
#include <iostream>

namespace librav{

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

using Position2D = position2d<uint32_t>;
using Position2Dd = position2d<double>;
using Position3D = position3d<uint32_t>;
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

#endif /* SRC_MAP_COMMON_TYPES_H_ */
