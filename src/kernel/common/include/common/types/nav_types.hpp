/* 
 * nav_types.hpp
 * 
 * Created on: Oct 10, 2018 11:48
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef NAV_TYPES_HPP
#define NAV_TYPES_HPP

#include <cstdint>
#include <vector>
#include <iostream>

#include "eigen3/Eigen/Core"

#include "common/types/base_types.hpp"

namespace librav
{
using Position2Di = value2d<int32_t>;
using Position2Dd = value2d<double>;
using Position3Di = value3d<int32_t>;
using Position3Dd = value3d<double>;

using Velocity2Di = value2d<int32_t>;
using Velocity2Dd = value2d<double>;
using Velocity3Di = value3d<int32_t>;
using Velocity3Dd = value3d<double>;

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

template <typename T>
struct Range2D
{
	T min;
	T max;
};

template <typename T>
struct BoundingBox
{
	Range2D<T> x;
	Range2D<T> y;
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
} // namespace librav

#endif /* NAV_TYPES_HPP */
