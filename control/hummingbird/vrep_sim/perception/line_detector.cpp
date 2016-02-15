/*
 * line_detector.cpp
 *
 *  Created on: Aug 6, 2015
 *      Author: rdu
 */

#include "perception/line_detector.h"

using namespace srcl_ctrl;

LineDetector::LineDetector(void)
{

}

LineDetector::~LineDetector(void)
{

}

void LineDetector::BinarizeImage(unsigned char mono_image[IMG_RES_Y][IMG_RES_X])
{
	int row, col;

	for(row = 0; row < 90; row++)
	{
		for(col = 0; col < 160; col++)
		{
			if(mono_image[row][col] > bin_threshold_)
				bin_image_[row][col] = 255;
			else
				bin_image_[row][col] = 0;
		}
	}
}

void LineDetector::ExtractRefLine()
{
	int row, col;

	for(row = 0; row < 90; row++)
	{
		unsigned char right_index;
		unsigned char left_index;

		for(col = 0; col < 80; col++)
		{
			if(bin_image_[row][80-col] > bin_image_[row][80-col-1])
			{
				ref_line_[row][80-col] = 0;
				left_index = 81-col;
			}
		}

		for(col = 81; col < 159; col++)
		{
			if(bin_image_[row][col] > bin_image_[row][col+1])
			{
				ref_line_[row][col] = 0;
				right_index = col;
			}
		}

		if((right_index - left_index)%2 == 0)
			bin_image_[row][(right_index - left_index)/2] = 0;
	}
}
