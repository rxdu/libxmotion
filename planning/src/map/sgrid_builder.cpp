/*
 * sgrid_builder.cpp
 *
 *  Created on: Mar 23, 2016
 *      Author: rdu
 */

#include "sgrid_builder.h"

using namespace srcl_ctrl;
using namespace cv;

SGridBuilder::SGridBuilder()
{

}

SGridBuilder::~SGridBuilder()
{

}

OccupancyType SGridBuilder::CheckAreaOccupancy(Mat img, BoundingBox area)
{
	Range rngx(area.x.min,area.x.max+1);
	Range rngy(area.y.min, area.y.max+1);

	// Attention:
	//	Points and Size go (x,y); (width,height) ,- Mat has (row,col).
	Mat checked_area = img(rngy,rngx);

	unsigned long free_points = 0;
	unsigned long occupied_points = 0;
	OccupancyType type;

	for(int i = 0; i < checked_area.cols; i++)
		for(int j = 0; j < checked_area.rows; j++)
		{
			if(checked_area.at<uchar>(Point(i,j)) > 0)
				free_points++;
			else
				occupied_points++;

			if(occupied_points !=0 && free_points != 0)
			{
				type = OccupancyType::MIXED;
				break;
			}
		}

	if(free_points !=0 && occupied_points == 0)
		type = OccupancyType::FREE;

	if(free_points ==0 && occupied_points != 0)
		type = OccupancyType::OCCUPIED;

	return type;
}

SquareGrid* SGridBuilder::BuildSquareGrid(cv::InputArray _src, uint32_t width, uint32_t height)
{
	Mat image_bin;
	Mat src = _src.getMat();

	// Binarize grayscale image
	threshold(src, image_bin, 200, 255, THRESH_BINARY);

	// Create quadtree
//	uint32_t cell_size = image_bin.cols /
	SquareGrid *sgrid = new SquareGrid(height,width, 5);

	return sgrid;
}
