/*
 * vis_utils.h
 *
 *  Created on: Aug 3, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_VIS_VIS_UTILS_H_
#define PLANNING_SRC_VIS_VIS_UTILS_H_

#include <cstdint>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "common/planning_types.h"

namespace srcl_ctrl {

typedef struct {
    double r;       // percent
    double g;       // percent
    double b;       // percent
} RGBColor;

typedef struct {
    double h;       // angle in degrees
    double s;       // percent
    double v;       // percent
} HSVColor;

namespace VisUtils {
	extern cv::Scalar default_pt_color_;		// default point color
	extern cv::Scalar default_ln_color_;		// default line color
	extern cv::Scalar default_area_color_;		// default area color

	extern cv::Scalar bk_color_;		// background color
	extern cv::Scalar ln_color_;		// line color
	extern cv::Scalar obs_color_;		// obstacle color
	extern cv::Scalar aoi_color_;		// area of interest color
	extern cv::Scalar start_color_; 	// starting cell color
	extern cv::Scalar finish_color_;	// finishing cell color

	void DrawPoint(cv::Mat img, cv::Point pos, const cv::Scalar& color = VisUtils::default_pt_color_);
	void DrawLine(cv::Mat img, cv::Point pt1, cv::Point pt2, const cv::Scalar& color = VisUtils::default_ln_color_);
	void DrawArrow(cv::Mat img, cv::Point pos, double angle, const cv::Scalar& color = VisUtils::default_pt_color_);

	void FillRectangularArea(cv::Mat img, BoundingBox bbox, const cv::Scalar& color = VisUtils::default_area_color_);

	// RGB: r,g,b ∈ [0-255]
	// HSV: h ∈ [0, 360] and s,v ∈ [0, 1]
	HSVColor RGB2HSV(RGBColor in);
	RGBColor HSV2RGB(HSVColor in);
};

}

#endif /* PLANNING_SRC_VIS_VIS_UTILS_H_ */
