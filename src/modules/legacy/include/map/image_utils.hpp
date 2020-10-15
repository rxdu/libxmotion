/* 
 * image_utils.hpp
 * 
 * Created on: Mar 24, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef IMAGE_UTILS_HPP
#define IMAGE_UTILS_HPP

#include <cstdint>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "adtypes/adtypes.hpp"

namespace ivnav
{

typedef struct
{
	int16_t top;
	int16_t bottom;
	int16_t left;
	int16_t right;
} PaddingSize;

/*
 * OpenCV Image Coordinate System:
 *
 * 	origin ------------------> x
 *		|
 *		|
 *  	|
 *		|
 *		|
 *		|
 *		v
 *		y
 *		
 */
namespace Map
{
void BinarizeImage(cv::InputArray _src, cv::OutputArray _dst, uint8_t thresh);
PaddingSize PadImageToSquared(cv::InputArray _src, cv::OutputArray _dst);
PaddingSize PadImageTo2Exp(cv::InputArray _src, cv::OutputArray _dst);
void ExpandObstacleAreaOnImage(cv::InputArray _src, cv::OutputArray _dst, int16_t expand_size);

void CreateOccupancyMapForRRT(uint64_t width, uint64_t height, cv::OutputArray _dst);

OccupancyType CheckAreaOccupancy(cv::InputArray _src, BoundingBox<int32_t> area);
bool IsPointNonObstacle(cv::InputArray _src, cv::Point pt);
};
}

#endif /* IMAGE_UTILS_HPP */
