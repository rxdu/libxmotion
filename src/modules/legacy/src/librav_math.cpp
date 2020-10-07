/* 
 * librav_math.cpp
 * 
 * Created on: Nov 06, 2017 14:41
 * Description:   
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#include "map/librav_math.hpp"

using namespace autodrive;

/**
 * frames: base_frame, frame1
 * pos: the interested position defined in frame1
 * transform: the orientation of frame 1 defined in base_frame
 * return value: the interested position defined in base_frame
 */
Position3Dd TransMath::TransformPosition3D(Transform3D transform, Position3Dd pos)
{
	Eigen::Vector3d output = transform.trans * transform.quat * Eigen::Vector3d(pos.x, pos.y, pos.z);

	return Position3Dd(output[0], output[1], output[2]);
}
