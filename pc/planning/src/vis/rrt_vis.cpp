/*
 * rrt_vis.cpp
 *
 *  Created on: Aug 3, 2016
 *      Author: rdu
 */

#include "vis/rrt_vis.h"
#include "vis/vis_utils.h"
#include "map/map_utils.h"

using namespace srcl_ctrl;
using namespace cv;

cv::Scalar RRTVis::bk_color_ = Scalar(255,255,255);
cv::Scalar RRTVis::ln_color_ = Scalar(Scalar(0,0,0));
cv::Scalar RRTVis::obs_color_ = Scalar(Scalar(0,102,204));
cv::Scalar RRTVis::aoi_color_ = Scalar(Scalar(0,255,255));
cv::Scalar RRTVis::start_color_ = Scalar(0,0,255);
cv::Scalar RRTVis::finish_color_ = Scalar(153,76,0);

void RRTVis::VisRRTPath(const std::vector<Position2Dd>& path, MapInfo info, cv::InputArray _src, cv::OutputArray _dst)
{
	Mat src_img_color;
	cvtColor(_src, src_img_color, CV_GRAY2BGR);
	_dst.create(src_img_color.size(), src_img_color.type());
	Mat dst = _dst.getMat();

	for(auto& wp : path)
	{
		Position2D pos;
		pos = MapUtils::CoordinatesFromWorldToMap(wp, info);
		VisUtils::DrawPoint(src_img_color, Point(pos.x, pos.y));
	}

	src_img_color.copyTo(dst);
}


