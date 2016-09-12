/*
 * transformation.cpp
 *
 *  Created on: Sep 10, 2016
 *      Author: rdu
 */

#include <iostream>

#include "transformation.h"

using namespace Eigen;

using namespace srcl_ctrl;
using namespace srcl_ctrl::utils;

/**
 * frames: base_frame, frame1
 * pos: the interested position defined in frame1
 * transform: the orientation of frame 1 defined in base_frame
 * return value: the interested position defined in base_frame
 */
Position3Dd Transformation::TransformPosition3D(Transform3D transform, Position3Dd pos)
{
	//Vector3d input(pos.x, pos.y, pos.z);

	//Vector3d output = transform.quat.conjugate() * transform.trans.inverse() * input;
	Vector3d output = transform.trans * transform.quat * Vector3d(pos.x, pos.y, pos.z);

//	static bool flag = true;
//	if(flag)
//	{
//		std::cout << "transform:\n " << "input: " <<  pos.x << " , " << pos.y << " , " << pos.z << std::endl;
//		std::cout << "output: " << output[0] << " , " << output[1] << " , " << output[2] << std::endl;
//		flag = false;
//	}

	return Position3Dd(output[0], output[1], output[2]);
}


