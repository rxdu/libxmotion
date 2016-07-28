/*
 * maputils_unittest.cpp
 *
 *  Created on: Jul 28, 2016
 *      Author: rdu
 */

#include "gtest/gtest.h"

// c++
#include <iostream>
#include <stdio.h>
#include <vector>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "common/planning_types.h"
#include "map/map_utils.h"
#include "map/map_type.h"
#include "map/graph_builder.h"
#include "map/sgrid_builder.h"

using namespace cv;
using namespace srcl_ctrl;

struct MapUtilsTest: testing::Test
{
	Mat input_image;
	Map_t<SquareGrid> sgrid_map;
	MapInfo minfo;

	MapUtilsTest()
	{
		input_image = imread( "/home/rdu/Workspace/srcl_robot_suite/srcl_ctrl/pc/planning/data/example.png", IMREAD_GRAYSCALE );
		sgrid_map = SGridBuilder::BuildSquareGridMap(input_image, 32);
		sgrid_map.info.world_size_x = 10.0;
		sgrid_map.info.world_size_y = 5.0;
		sgrid_map.info.scale_x = static_cast<double>(sgrid_map.info.map_size_x)/sgrid_map.info.world_size_x;
		sgrid_map.info.scale_y = static_cast<double>(sgrid_map.info.map_size_y)/sgrid_map.info.world_size_y;

		minfo = sgrid_map.info;
	}

	virtual ~MapUtilsTest()
	{
	}
};

//static Position2Dd CoordinatesFromMapToWorld(Position2D map_pos, MapInfo info);
//static Position2D CoordinatesFromWorldToMap(Position2Dd map_pos, MapInfo info);
//static Position2D CoordinatesFromPaddedToOriginal(Position2D pad_pos, MapInfo info);
//static Position2D CoordinatesFromOriginalToPadded(Position2D ori_pos, MapInfo info);

TEST_F(MapUtilsTest, MapToWorld)
{
	Position2D input;
	Position2Dd exp_value;
	Position2Dd calc_value;

	// calculation 1
	input.x = 0;
	input.y = 0;
	exp_value.x = 0.0;
	exp_value.y = 0.0;
	calc_value = MapUtils::CoordinatesFromMapToWorld(input, minfo);

	EXPECT_EQ(exp_value, calc_value);

	// calculation 2
	input.x = minfo.map_size_x;
	input.y = minfo.map_size_y;
	exp_value.x = minfo.world_size_x;
	exp_value.y = minfo.world_size_y;
	calc_value = MapUtils::CoordinatesFromMapToWorld(input, minfo);

	EXPECT_EQ(exp_value, calc_value);

	// calculation 3
	input.x = minfo.map_size_x + 1;
	input.y = minfo.map_size_y + 1;
	exp_value.x = minfo.world_size_x;
	exp_value.y = minfo.world_size_y;
	calc_value = MapUtils::CoordinatesFromMapToWorld(input, minfo);

	EXPECT_EQ(exp_value, calc_value);

	// calculation 4
	input.x = minfo.map_size_x/2;
	input.y = minfo.map_size_y/2;
	exp_value.x = minfo.world_size_x/2;
	exp_value.y = minfo.world_size_y/2;
	calc_value = MapUtils::CoordinatesFromMapToWorld(input, minfo);

	EXPECT_EQ(exp_value, calc_value);
}

