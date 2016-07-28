/*
 * common_types.h
 *
 *  Created on: Jan 28, 2016
 *      Author: rdu
 */

#ifndef SRC_MAP_COMMON_TYPES_H_
#define SRC_MAP_COMMON_TYPES_H_

#include <cstdint>

namespace srcl_ctrl{

typedef struct _postion2d
{
	uint32_t x;
	uint32_t y;

	bool operator==(const struct _postion2d& other) const
	{
		if(this->x == other.x && this->y == other.y)
			return true;
		else
			return false;
	}
}Position2D;

typedef struct _position2dd
{
	double x;
	double y;

	bool operator==(const struct _position2dd& other) const
	{
		if(this->x == other.x && this->y == other.y)
			return true;
		else
			return false;
	}
}Position2Dd;

typedef struct _position3d
{
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

}

#endif /* SRC_MAP_COMMON_TYPES_H_ */
