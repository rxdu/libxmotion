/*
 * test_image.cpp
 *
 *  Created on: Mar 24, 2016
 *      Author: rdu
 */

// standard libaray
#include <stdio.h>
#include <vector>

#include "map/map2d.hpp"
#include "fastplot/fastplot.hpp"

using namespace cv;
using namespace librav;

int main(int argc, char** argv )
{	
	Map2D map;

	map.LoadMapImage("/home/rdu/Workspace/librav/data/example.png");
	map.IsMapValid();
	map.SetWorldSize(20,10);
	map.IsMapValid();
	
	FastPlot::ShowImage(map.raw_image_, "test color");


// 	if ( !image_raw.data )
// 	{
// 		printf("No image data \n");
// 		return -1;
// 	}

// //	ImageUtils image_utils;
// //	image_utils.BinarizeImage(image_raw, image_disp,200);

// 	Mat image_bin;
// 	ImageUtils::BinarizeImage(image_raw, image_bin,200);
// //	ImageUtils::PadImageToSquared(image_bin, image_disp);
// 	ImageUtils::PadImageTo2Exp(image_bin, image_disp);

// 	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
// 	imshow("Processed Image", image_disp);

// 	waitKey(0);

// 	imwrite( "test_image.jpg", image_disp);

	return 0;
}
