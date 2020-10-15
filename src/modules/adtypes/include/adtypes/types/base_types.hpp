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

namespace ivnav
{

using TimeStamp = uint64_t;
using CTimeStamp = double;

template <typename T>
struct value2
{
	value2() : x(0), y(0) {}
	value2(T _x, T _y) : x(_x), y(_y) {}

	T x;
	T y;

	bool operator==(const struct value2 &other) const
	{
		if (this->x == other.x && this->y == other.y)
			return true;
		else
			return false;
	}

	friend std::ostream &operator<<(std::ostream &os, const struct value2 &pos)
	{
		os << pos.x << " , " << pos.y;
		return os;
	}
};

template <typename T>
struct value3
{
	value3() : x(0), y(0), z(0) {}
	value3(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}

	T x;
	T y;
	T z;

	bool operator==(const struct value3 &other) const
	{
		if (this->x == other.x && this->y == other.y && this->z == other.z)
			return true;
		else
			return false;
	}

	friend std::ostream &operator<<(std::ostream &os, const struct value3 &pos)
	{
		os << pos.x << " , " << pos.y << " , " << pos.z;
		return os;
	}
};

using Point2f = value2<float>;
using Point2d = value2<double>;
using Point2i = value2<int32_t>;

using Point3f = value3<float>;
using Point3d = value3<double>;
using Point3i = value3<int32_t>;

using CovarMatrix2d = Eigen::Matrix<double, 2, 2>;

} // namespace ivnav

#endif /* BASE_TYPES_HPP */
