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

namespace srcl_ctrl{

typedef struct _postion2d
{
	uint32_t x;
	uint32_t y;

	_postion2d():x(0),y(0){}
	_postion2d(uint32_t _x, uint32_t _y):x(_x),y(_y){}
	//_postion2d(const _postion2d& other): x(other.x), y(other.y){}
//	_postion2d& operator=(_postion2d other)
//	{
//		return _postion2d(other.x, other.y);
//	}

	bool operator==(const struct _postion2d& other) const
	{
		if(this->x == other.x && this->y == other.y)
			return true;
		else
			return false;
	}

	// http://stackoverflow.com/questions/236801/should-operator-be-implemented-as-a-friend-or-as-a-member-function
	friend std::ostream& operator<<(std::ostream& os, const struct _postion2d& pos)
	{
		os << pos.x << " , " << pos.y;
		return os;
	}
}Position2D;

typedef struct _position2dd
{
	_position2dd():x(0),y(0){}
	_position2dd(double _x, double _y):x(_x),y(_y){}

	double x;
	double y;

	bool operator==(const struct _position2dd& other) const
	{
		if(this->x == other.x && this->y == other.y)
			return true;
		else
			return false;
	}

	friend std::ostream& operator<<(std::ostream& os, const struct _position2dd& pos)
	{
		os << pos.x << " , " << pos.y;
		return os;
	}
}Position2Dd;

typedef struct _position3dd
{
	_position3dd():x(0),y(0),z(0){}
	_position3dd(double _x, double _y, double _z):x(_x),y(_y),z(_z){}

	double x;
	double y;
	double z;

	bool operator==(const struct _position3dd& other) const
	{
		if(this->x == other.x && this->y == other.y && this->z == other.z)
			return true;
		else
			return false;
	}

	friend std::ostream& operator<<(std::ostream& os, const struct _position3dd& pos)
	{
	    os << pos.x << " , " << pos.y << " , " << pos.z;
	    return os;
	}
}Position3Dd;

typedef struct _position3d
{
	_position3d():x(0),y(0),z(0){}
	_position3d(uint32_t _x, uint32_t _y, uint32_t _z):x(_x),y(_y),z(_z){}

	uint32_t x;
	uint32_t y;
	uint32_t z;

	bool operator==(const struct _position3d& other) const
	{
		if(this->x == other.x && this->y == other.y && this->z == other.z)
			return true;
		else
			return false;
	}

	friend std::ostream& operator<<(std::ostream& os, const struct _position3d& pos)
	{
		os << pos.x << " , " << pos.y << " , " << pos.z;
		return os;
	}
}Position3D;

enum class OccupancyType
{
	FREE,
	OCCUPIED,
	MIXED,
	INTERESTED
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
} KeyframeSet;

}

#endif /* SRC_MAP_COMMON_TYPES_H_ */
