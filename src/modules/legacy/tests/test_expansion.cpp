/*
 * test_expansion.cpp
 *
 *  Created on: Nov 8, 2016
 *      Author: rdu
 */

// standard libaray
#include <stdio.h>
#include <vector>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "map/image_utils.h"

using namespace cv;
using namespace ivnav;

int main(int argc, char** argv )
{
	std::string img_paht = "/home/rdu/Workspace/srcl_rtk/librav/pc/planning/data/experiments/set2/map_path_repair.png";

	Mat image_raw;
	Mat image_disp;
	image_raw = imread( img_paht, IMREAD_GRAYSCALE );

	if ( !image_raw.data )
	{
		printf("No image data \n");
		return -1;
	}

//	ImageUtils image_utils;
//	image_utils.BinarizeImage(image_raw, image_disp,200);

	Mat image_bin;
	ImageUtils::BinarizeImage(image_raw, image_bin,200);
//	ImageUtils::PadImageToSquared(image_bin, image_disp);
	Mat image_exp;
	ImageUtils::ExpandObstacleAreaOnImage(image_bin, image_exp, 20);

//	ImageUtils::PadImageTo2Exp(image_bin, image_disp);
//
//	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
//	imshow("Processed Image", image_disp);

	namedWindow("Processed Image 2", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image 2", image_exp);

	waitKey(0);

	//imwrite( "test_image.jpg", image_disp);

	return 0;
}
