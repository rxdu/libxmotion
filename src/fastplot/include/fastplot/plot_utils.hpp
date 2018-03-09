/* 
 * plot_utils.h
 * 
 * Created on: Dec 29, 2017 18:11
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef PLOT_UTILS_H
#define PLOT_UTILS_H

#include <cstdint>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "common/librav_types.hpp"

namespace librav
{

struct RGBColor
{
	double r; // percent
	double g; // percent
	double b; // percent
};

struct HSVColor
{
	double h; // angle in degrees
	double s; // percent
	double v; // percent
};

struct FastplotColors
{
	static const cv::Scalar default_pt_color;   // default point color
	static const cv::Scalar default_ln_color;   // default line color
	static const cv::Scalar default_area_color; // default area color

	static const cv::Scalar bg_color;	 // background color
	static const cv::Scalar ln_color;	 // line color
	static const cv::Scalar obs_color;	// obstacle color
	static const cv::Scalar aoi_color;	// area of interest color
	static const cv::Scalar start_color;  // starting cell color
	static const cv::Scalar intermediate_color;  // intermediate cell color
	static const cv::Scalar finish_color; // finishing cell color
};

namespace PlotUtils
{

void DrawPoint(cv::Mat img, cv::Point pos, const cv::Scalar &color = FastplotColors::default_pt_color);
void DrawLine(cv::Mat img, cv::Point pt1, cv::Point pt2, const cv::Scalar &color = FastplotColors::default_ln_color);
void DrawArrow(cv::Mat img, cv::Point base_pos, double length, double angle, const cv::Scalar &color = FastplotColors::default_pt_color);
void DrawSquare(cv::Mat img, BoundingBox<int32_t> bbox, const cv::Scalar &color = FastplotColors::default_area_color);

// RGB: r,g,b ∈ [0-255]
// HSV: h ∈ [0, 360] and s,v ∈ [0, 1]
HSVColor RGB2HSV(RGBColor in);
RGBColor HSV2RGB(HSVColor in);

std::string GetMatDepth(const cv::Mat &mat);
std::string GetMatType(const cv::Mat &mat);
};
}

#endif /* PLOT_UTILS_H */
