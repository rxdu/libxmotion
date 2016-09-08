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
		sgrid_map.info.SetWorldSize(10.0, 5.0);

		minfo = sgrid_map.info;
	}

	virtual ~MapUtilsTest()
	{
	}
};

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
	calc_value = MapUtils::CoordinatesFromMapToMapWorld(input, minfo);

	EXPECT_EQ(exp_value, calc_value);

	// calculation 2
	input.x = minfo.map_size_x;
	input.y = minfo.map_size_y;
	exp_value.x = minfo.world_size_x;
	exp_value.y = minfo.world_size_y;
	calc_value = MapUtils::CoordinatesFromMapToMapWorld(input, minfo);

	EXPECT_EQ(exp_value, calc_value);

	// calculation 3
	input.x = minfo.map_size_x + 1;
	input.y = minfo.map_size_y + 1;
	exp_value.x = minfo.world_size_x;
	exp_value.y = minfo.world_size_y;
	calc_value = MapUtils::CoordinatesFromMapToMapWorld(input, minfo);

	EXPECT_EQ(exp_value, calc_value);

	// calculation 4
	input.x = minfo.map_size_x/2;
	input.y = minfo.map_size_y/2;
	exp_value.x = minfo.world_size_x/2;
	exp_value.y = minfo.world_size_y/2;
	calc_value = MapUtils::CoordinatesFromMapToMapWorld(input, minfo);

	//EXPECT_EQ(exp_value, calc_value);
	EXPECT_LT(exp_value.x - calc_value.x , 0.1);
	EXPECT_LT(exp_value.y - calc_value.y , 0.1);
}

TEST_F(MapUtilsTest, WorldToMap)
{
	Position2Dd input;
	Position2D exp_value;
	Position2D calc_value;

	//  calculation 1
	input.x = 0.0;
	input.y = 0.0;
	exp_value.x = 0;
	exp_value.y = 0;
	calc_value = MapUtils::CoordinatesFromMapWorldToMap(input, minfo);

	EXPECT_EQ(exp_value, calc_value);

	//  calculation 2
	input.x = minfo.world_size_x;
	input.y = minfo.world_size_y;
	exp_value.x = minfo.map_size_x;
	exp_value.y = minfo.map_size_y;
	calc_value = MapUtils::CoordinatesFromMapWorldToMap(input, minfo);

	EXPECT_EQ(exp_value, calc_value);

	//  calculation 3
	input.x = minfo.world_size_x+1;
	input.y = minfo.world_size_y+1;
	exp_value.x = minfo.map_size_x;
	exp_value.y = minfo.map_size_y;
	calc_value = MapUtils::CoordinatesFromMapWorldToMap(input, minfo);

	EXPECT_EQ(exp_value, calc_value);

	//  calculation 4
	input.x = minfo.world_size_x/2;
	input.y = minfo.world_size_y/2;
	exp_value.x = minfo.map_size_x/2;
	exp_value.y = minfo.map_size_y/2;
	calc_value = MapUtils::CoordinatesFromMapWorldToMap(input, minfo);

	EXPECT_EQ(exp_value, calc_value);
}

TEST_F(MapUtilsTest, PaddedToOriginal)
{
	Position2D input;
	Position2D exp_value;
	Position2D calc_value;

	//  calculation 1
	input.x = 0.0;
	input.y = 0.0;
	exp_value.x = 0;
	exp_value.y = 0;
	calc_value = MapUtils::CoordinatesFromPaddedToOriginal(input, minfo);

	EXPECT_EQ(exp_value, calc_value);

	//  calculation 2
	input.x = minfo.padded_left;
	input.y = minfo.padded_top;
	exp_value.x = 0;
	exp_value.y = 0;
	calc_value = MapUtils::CoordinatesFromPaddedToOriginal(input, minfo);

	EXPECT_EQ(exp_value, calc_value);

	//  calculation 3
	input.x = minfo.map_size_x + minfo.padded_left;
	input.y = minfo.map_size_y + minfo.padded_top;
	exp_value.x = minfo.map_size_x;
	exp_value.y = minfo.map_size_y;
	calc_value = MapUtils::CoordinatesFromPaddedToOriginal(input, minfo);

	EXPECT_EQ(exp_value, calc_value);

	//  calculation 4
	input.x = minfo.map_size_x + minfo.padded_left + minfo.padded_right;
	input.y = minfo.map_size_y + minfo.padded_top + minfo.padded_bottom;
	exp_value.x = minfo.map_size_x;
	exp_value.y = minfo.map_size_y;
	calc_value = MapUtils::CoordinatesFromPaddedToOriginal(input, minfo);

	EXPECT_EQ(exp_value, calc_value);
}

TEST_F(MapUtilsTest, OriginalToPadded)
{
	Position2D input;
	Position2D exp_value;
	Position2D calc_value;

	//  calculation 1
	input.x = 0.0;
	input.y = 0.0;
	exp_value.x = minfo.padded_left;
	exp_value.y = minfo.padded_top;
	calc_value = MapUtils::CoordinatesFromOriginalToPadded(input, minfo);

	EXPECT_EQ(exp_value, calc_value);

	//  calculation 2
	input.x = minfo.map_size_x;
	input.y = minfo.map_size_y;
	exp_value.x = minfo.map_size_x + minfo.padded_left;
	exp_value.y = minfo.map_size_y + minfo.padded_top;
	calc_value = MapUtils::CoordinatesFromOriginalToPadded(input, minfo);

	EXPECT_EQ(exp_value, calc_value);

//	//  calculation 3
//	input.x = minfo.map_size_x/2;
//	input.y = minfo.map_size_y/2;
//	exp_value.x = (minfo.map_size_x + minfo.padded_left + minfo.padded_right)/2;
//	exp_value.y = (minfo.map_size_y + minfo.padded_top + minfo.padded_bottom)/2 - 1; // -1 is to compensate the roundup error
//	calc_value = MapUtils::CoordinatesFromOriginalToPadded(input, minfo);
//	EXPECT_EQ(exp_value, calc_value);
}

