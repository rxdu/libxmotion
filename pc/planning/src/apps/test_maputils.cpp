/*
 * test_maputils.cpp
 *
 *  Created on: Jul 28, 2016
 *      Author: rdu
 */

// c++
#include <iostream>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "map/map_utils.h"
#include "map/map_type.h"
#include "map/graph_builder.h"
#include "map/sgrid_builder.h"

using namespace cv;
using namespace srcl_ctrl;

int main(int argc, char** argv)
{
	Mat input_image;
	Map_t<SquareGrid> sgrid_map;

	input_image = imread( "/home/rdu/Workspace/srcl_robot_suite/srcl_ctrl/pc/planning/data/example.png", IMREAD_GRAYSCALE );
	sgrid_map = SGridBuilder::BuildSquareGridMap(input_image, 32);
	sgrid_map.info.world_size_x = 10.0;
	sgrid_map.info.world_size_y = 5.0;

	std::cout << "-----------------------------------------" << std::endl;
	std::cout << "* input data info " << std::endl;
	std::cout << " -- map size: " << sgrid_map.info.map_size_x << "," << sgrid_map.info.map_size_y << std::endl;
	std::cout << " -- padding size (l-r-t-b): "
			<< sgrid_map.info.padded_left << ","
			<< sgrid_map.info.padded_right << ","
			<< sgrid_map.info.padded_top << ","
			<< sgrid_map.info.padded_bottom << std::endl;
	std::cout << " -- world size: " << sgrid_map.info.world_size_x << "," << sgrid_map.info.world_size_y << std::endl;

	std::cout << "\n* conversion result " << std::endl;
	std::cout << " -- map -> world: " << "(0, 0)" << std::endl;

//	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
//	imshow("Processed Image", sgrid_map.padded_image);
//
//	waitKey(0);
}

