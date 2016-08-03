/*
 * image_utils.h
 *
 *  Created on: Mar 24, 2016
 *      Author: rdu
 */

#ifndef SRC_MAP_IMAGE_UTILS_H_
#define SRC_MAP_IMAGE_UTILS_H_

#include <cstdint>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "common/planning_types.h"

namespace srcl_ctrl
{

typedef struct {
	int16_t top;
	int16_t bottom;
	int16_t left;
	int16_t right;
} PaddingSize;

class ImageUtils{
public:
	ImageUtils(){};
	~ImageUtils(){};

public:
	static void BinarizeImage(cv::InputArray _src, cv::OutputArray _dst, uint8_t thresh);
	static PaddingSize PadImageToSquared(cv::InputArray _src, cv::OutputArray _dst);
	static PaddingSize PadImageTo2Exp(cv::InputArray _src, cv::OutputArray _dst);

	static OccupancyType CheckAreaOccupancy(cv::InputArray _src, BoundingBox area);
	static bool IsPointOccupied(cv::InputArray _src, cv::Point pt);
};

}

#endif /* SRC_MAP_IMAGE_UTILS_H_ */
