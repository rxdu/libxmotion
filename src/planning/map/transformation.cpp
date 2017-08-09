/*
 * transformation.cpp
 *
 *  Created on: Sep 10, 2016
 *      Author: rdu
 */

#include <iostream>

#include "map/transformation.h"

using namespace Eigen;

using namespace librav;
using namespace librav::utils;

/**
 * frames: base_frame, frame1
 * pos: the interested position defined in frame1
 * transform: the orientation of frame 1 defined in base_frame
 * return value: the interested position defined in base_frame
 */
Position3Dd Transformation::TransformPosition3D(Transform3D transform, Position3Dd pos)
{
	Vector3d output = transform.trans * transform.quat * Vector3d(pos.x, pos.y, pos.z);

	return Position3Dd(output[0], output[1], output[2]);
}
