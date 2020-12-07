/*
 * typesize_unittest.cpp
 *
 *  Created on: Jul 21, 2016
 *      Author: rdu
 */

#include <stdio.h>
#include <vector>

#include "gtest/gtest.h"

#include "common/librav_math.hpp"

using namespace Eigen;
using namespace robotnav;
using namespace robotnav::utils;
using namespace robotnav::utils::Transformation;

TEST(FrameTest, Transformation)
{
	Position3Dd pt_input;
	Position3Dd pt_output;

	pt_input.x = 1;
	pt_input.y = 1;
	pt_input.z = 1;

	Transform3D transform;

	//transform.trans = Translation3D(0,0,0);
	transform.trans = Translation3D(1,1,1);

	Eigen::Quaterniond rot(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()));
	transform.quat = rot;

	pt_output = TransformPosition3D(transform, pt_input);

	std::cout << "trans input: " << pt_input.x << " , " << pt_input.y << " , " << pt_input.z << std::endl;
	std::cout << "trans result: " << pt_output.x << " , " << pt_output.y << " , " << pt_output.z << std::endl;
}
