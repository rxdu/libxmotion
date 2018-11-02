/* 
 * base_types.hpp
 * 
 * Created on: Oct 10, 2018 11:44
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef BASE_TYPES_HPP
#define BASE_TYPES_HPP

#include <cstdint>
#include <vector>
#include <iostream>

#include "eigen3/Eigen/Core"

namespace librav
{

using TimeStamp = uint64_t;
using CTimeStamp = double;

template <typename T>
struct value2d
{
	value2d() : x(0), y(0) {}
	value2d(T _x, T _y) : x(_x), y(_y) {}

	T x;
	T y;

	bool operator==(const struct value2d &other) const
	{
		if (this->x == other.x && this->y == other.y)
			return true;
		else
			return false;
	}

	friend std::ostream &operator<<(std::ostream &os, const struct value2d &pos)
	{
		os << pos.x << " , " << pos.y;
		return os;
	}
};

template <typename T>
struct value3d
{
	value3d() : x(0), y(0), z(0) {}
	value3d(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}

	T x;
	T y;
	T z;

	bool operator==(const struct value3d &other) const
	{
		if (this->x == other.x && this->y == other.y && this->z == other.z)
			return true;
		else
			return false;
	}

	friend std::ostream &operator<<(std::ostream &os, const struct value3d &pos)
	{
		os << pos.x << " , " << pos.y << " , " << pos.z;
		return os;
	}
};

using Point2f = value2d<float>;
using Point2d = value2d<double>;
using Point2i = value2d<int32_t>;

using Point3f = value3d<float>;
using Point3d = value3d<double>;
using Point3i = value3d<int32_t>;

} // namespace librav

#endif /* BASE_TYPES_HPP */
