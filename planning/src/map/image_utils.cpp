/*
 * image_utils.cpp
 *
 *  Created on: Mar 24, 2016
 *      Author: rdu
 */

#include "image_utils.h"

using namespace srcl_ctrl;
using namespace cv;

ImageUtils::ImageUtils()
{

}

ImageUtils::~ImageUtils()
{

}

void ImageUtils::BinarizeImage(cv::InputArray _src, cv::OutputArray _dst, uint8_t thresh)
{
	// Prepare data structures
	Mat src = _src.getMat();
	_dst.create(src.size(), CV_8UC1);
	Mat dst = _dst.getMat();

	// Binarize grayscale image
	threshold(src, dst, thresh, 255, THRESH_BINARY);
}

void ImageUtils::PadImageToSquared(cv::InputArray _src, cv::OutputArray _dst)
{

}

void ImageUtils::PadImageTo2N(cv::InputArray _src, cv::OutputArray _dst)
{

}


