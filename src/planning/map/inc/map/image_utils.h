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

#include "common/librav_types.hpp"

namespace librav
{

typedef struct {
	int16_t top;
	int16_t bottom;
	int16_t left;
	int16_t right;
} PaddingSize;

//class ImageUtils{
//public:
//	ImageUtils(){};
//	~ImageUtils(){};
//
//public:
//	static void BinarizeImage(cv::InputArray _src, cv::OutputArray _dst, uint8_t thresh);
//	static PaddingSize PadImageToSquared(cv::InputArray _src, cv::OutputArray _dst);
//	static PaddingSize PadImageTo2Exp(cv::InputArray _src, cv::OutputArray _dst);
//
//	static void CreateOccupancyMapForRRT(uint64_t width, uint64_t height, cv::OutputArray _dst);
//
//	static OccupancyType CheckAreaOccupancy(cv::InputArray _src, BoundingBox<int32_t> area);
//	static bool IsPointNonObstacle(cv::InputArray _src, cv::Point pt);
//};

namespace ImageUtils{

	void BinarizeImage(cv::InputArray _src, cv::OutputArray _dst, uint8_t thresh);
	PaddingSize PadImageToSquared(cv::InputArray _src, cv::OutputArray _dst);
	PaddingSize PadImageTo2Exp(cv::InputArray _src, cv::OutputArray _dst);
	void ExpandObstacleAreaOnImage(cv::InputArray _src, cv::OutputArray _dst, int16_t expand_size);

	void CreateOccupancyMapForRRT(uint64_t width, uint64_t height, cv::OutputArray _dst);

	OccupancyType CheckAreaOccupancy(cv::InputArray _src, BoundingBox<int32_t> area);
	bool IsPointNonObstacle(cv::InputArray _src, cv::Point pt);

};

}

#endif /* SRC_MAP_IMAGE_UTILS_H_ */
