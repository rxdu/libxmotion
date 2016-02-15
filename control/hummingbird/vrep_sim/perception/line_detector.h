/*
 * line_detector.h
 *
 *  Created on: Aug 6, 2015
 *      Author: rdu
 */

#ifndef PERCEPTION_LINE_DETECTOR_H_
#define PERCEPTION_LINE_DETECTOR_H_

#include "vrep_client/robot_datatypes.h"

namespace srcl_ctrl
{
class LineDetector
{
public:
	LineDetector();
	~LineDetector();

public:
	void GetCentralRefLine();

public:
	void BinarizeImage(unsigned char mono_image[IMG_RES_Y][IMG_RES_X]);
	void ExtractRefLine();

private:
	const unsigned char bin_threshold_ = 140;

public:
	unsigned char bin_image_[IMG_RES_Y][IMG_RES_X];
	unsigned char ref_line_[IMG_RES_Y][IMG_RES_X];

};
}

#endif /* PERCEPTION_LINE_DETECTOR_H_ */
