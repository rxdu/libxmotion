/*
 * test_image.cpp
 *
 *  Created on: Mar 24, 2016
 *      Author: rdu
 */

// standard libaray
#include <stdio.h>
#include <vector>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "image_utils.h"

using namespace cv;
using namespace srcl_ctrl;

int main(int argc, char** argv )
{
	if ( argc != 2 )
	{
		printf("usage: DisplayImage.out <Image_Path>\n");
		return -1;
	}

	Mat image_raw;
	Mat image_disp;
	image_raw = imread( argv[1], IMREAD_GRAYSCALE );

	if ( !image_raw.data )
	{
		printf("No image data \n");
		return -1;
	}

//	ImageUtils image_utils;
//	image_utils.BinarizeImage(image_raw, image_disp,200);

	ImageUtils::BinarizeImage(image_raw, image_disp,200);

	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", image_disp);

	waitKey(0);

	return 0;
}


