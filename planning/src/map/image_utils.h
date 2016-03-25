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

#include "common/common_types.h"

namespace srcl_ctrl
{

class ImageUtils{
public:
	ImageUtils(){};
	~ImageUtils(){};

public:
	static void BinarizeImage(cv::InputArray _src, cv::OutputArray _dst, uint8_t thresh);
	static bool PadImageToSquared(cv::InputArray _src, cv::OutputArray _dst);
	static bool PadImageTo2Exp(cv::InputArray _src, cv::OutputArray _dst);

	OccupancyType CheckAreaOccupancy(cv::InputArray _src, BoundingBox area);
};

}

#endif /* SRC_MAP_IMAGE_UTILS_H_ */
