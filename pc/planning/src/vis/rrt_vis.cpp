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

	int thickness = 3;
	int lineType = 8;

	for(auto it = path.begin(); it != path.end() - 1; it++)
	{
		Position2D pos, pos_next;
		pos = MapUtils::CoordinatesFromWorldToMap(*it, info);
		if(!info.vector_map)
			pos = MapUtils::CoordinatesFromOriginalToPadded(pos, info);
		pos_next = MapUtils::CoordinatesFromWorldToMap(*(it+1), info);
		if(!info.vector_map)
			pos_next = MapUtils::CoordinatesFromOriginalToPadded(pos_next, info);

		cv::line( src_img_color,
				Point(pos.x, pos.y), Point(pos_next.x, pos_next.y),
				//Scalar( 51, 255, 255 ),
				Scalar( 126, 169, 51 ),
				thickness,
				lineType);

		VisUtils::DrawPoint(src_img_color, Point(pos.x, pos.y));
	}

	Position2D pos;
	pos = MapUtils::CoordinatesFromWorldToMap(*(path.end() - 1), info);
	if(!info.vector_map)
		pos = MapUtils::CoordinatesFromOriginalToPadded(pos, info);
	VisUtils::DrawPoint(src_img_color, Point(pos.x, pos.y));

	src_img_color.copyTo(dst);
}

void RRTVis::VisRRTGraph(const Graph_t<RRTNode>& graph, MapInfo info, cv::InputArray _src, cv::OutputArray _dst)
{
	Mat src, dst;
	int src_type = _src.getMat().type();
	if(src_type == CV_8UC1)
	{
		cvtColor(_src, src, CV_GRAY2BGR);
		_dst.create(src.size(), src.type());
		dst = _dst.getMat();
	}
	else
	{
		src = _src.getMat();
		_dst.create(_src.size(), _src.type());
		dst = _dst.getMat();
		src.copyTo(dst);
	}

	// draw all vertices
	std::vector<Vertex<RRTNode>*> vertices;
	vertices = graph.GetGraphVertices();
	for(auto itv = vertices.begin(); itv != vertices.end(); itv++)
	{
		// current vertex center coordinate
		Position2D center_pos;
		center_pos = MapUtils::CoordinatesFromWorldToMap((*itv)->bundled_data_.position, info);
		if(!info.vector_map)
			center_pos = MapUtils::CoordinatesFromOriginalToPadded(center_pos, info);
		cv::Point center(center_pos.x, center_pos.y);
		VisUtils::DrawPoint(dst, center);
	}

	// draw all edges
	auto edges = graph.GetGraphUndirectedEdges();
	for(auto it = edges.begin(); it != edges.end(); it++)
	{
		Position2D pos1, pos2;

		pos1 = MapUtils::CoordinatesFromWorldToMap((*it).src_->bundled_data_.position, info);
		if(!info.vector_map)
			pos1 = MapUtils::CoordinatesFromOriginalToPadded(pos1, info);
		pos2 = MapUtils::CoordinatesFromWorldToMap((*it).dst_->bundled_data_.position, info);
		if(!info.vector_map)
			pos2 = MapUtils::CoordinatesFromOriginalToPadded(pos2, info);

		uint64_t x1,y1,x2,y2;
		x1 = pos1.x;
		y1 = pos1.y;
		x2 = pos2.x;
		y2 = pos2.y;

		VisUtils::DrawLine(dst, Point(x1,y1), Point(x2,y2));
	}
}

