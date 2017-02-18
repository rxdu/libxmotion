/*
 * graph_vis.h
 *
 *  Created on: Dec 15, 2015
 *      Author: rdu
 */

#ifndef SRC_VISUALIZER_GRAPH_VIS_H_
#define SRC_VISUALIZER_GRAPH_VIS_H_

#include <vector>

#include "opencv2/opencv.hpp"

#include "graph/graph.h"
#include "vis/vis_utils.h"

namespace srcl_ctrl {

namespace Vis
{
	// graph visualization
	template<class GraphNodeType>
	void VisGraph(const Graph_t<GraphNodeType*>& graph, cv::InputArray _src, cv::OutputArray _dst, bool show_id)
	{
		cv::Mat src, dst;
		int src_type = _src.getMat().type();
		if(src_type == CV_8UC1)
		{
			cv::cvtColor(_src, src, CV_GRAY2BGR);
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
		std::vector<Vertex_t<GraphNodeType*>*> vertices;
		vertices = graph.GetGraphVertices();
		for(auto itv = vertices.begin(); itv != vertices.end(); itv++)
		{
			cv::Point center((*itv)->bundled_data_->location_.x, (*itv)->bundled_data_->location_.y);
			VisUtils::DrawPoint(dst, center);

			// current vertex center coordinate
			uint64_t x1,y1,x2,y2;
			x1 = (*itv)->bundled_data_->location_.x;
			y1 = (*itv)->bundled_data_->location_.y;

			if(show_id) {
//				if((*itv)->bundled_data_->data_id_ % 2 == 0)
//				{
					std::string id = std::to_string((*itv)->bundled_data_->data_id_);
					cv::putText(dst, id ,cv::Point(x1,y1), CV_FONT_NORMAL, 0.5, cv::Scalar(204,204,102),1,1);
//				}
			}
		}

		// draw all edges
		auto edges = graph.GetGraphUndirectedEdges();
		for(auto it = edges.begin(); it != edges.end(); it++)
		{
			uint64_t x1,y1,x2,y2;
			x1 = (*it).src_->bundled_data_->location_.x;
			y1 = (*it).src_->bundled_data_->location_.y;
			x2 = (*it).dst_->bundled_data_->location_.x;
			y2 = (*it).dst_->bundled_data_->location_.y;

			VisUtils::DrawLine(dst, cv::Point(x1,y1), cv::Point(x2,y2));
		}
	}

	template<class GraphNodeType>
	void VisGraphPath(const Path_t<GraphNodeType*>& path, cv::InputArray _src, cv::OutputArray _dst)
	{
		cv::Mat src, dst;
		int src_type = _src.getMat().type();
		if(src_type == CV_8UC1)
		{
			cv::cvtColor(_src, src, CV_GRAY2BGR);
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

		// draw starting and finishing cell
		auto cell_s = path[0]->bundled_data_;
		uint64_t x,y;
		x = cell_s->location_.x;
		x = x - (cell_s->bbox_.x.max - cell_s->bbox_.x.min)/8;
		y = cell_s->location_.y;
		y = y + (cell_s->bbox_.y.max - cell_s->bbox_.y.min)/8;

		VisUtils::FillRectangularArea(dst, cell_s->bbox_, VisUtils::start_color_);
		cv::putText(dst, "S" ,cv::Point(x,y), CV_FONT_NORMAL, 1, cv::Scalar(0,0,0),1,1);

		auto cell_f = (*(path.end()-1))->bundled_data_;
		x = cell_f->location_.x;
		x = x - (cell_f->bbox_.x.max - cell_f->bbox_.x.min)/8;
		y = cell_f->location_.y;
		y = y + (cell_f->bbox_.y.max - cell_f->bbox_.y.min)/8;

		VisUtils::FillRectangularArea(dst, cell_f->bbox_, VisUtils::finish_color_);
		cv::putText(dst, "F" ,cv::Point(x,y), CV_FONT_NORMAL, 1, cv::Scalar(0,0,0),1,1);

		// draw path
		uint64_t x1,y1,x2,y2;
		int thickness = 3;
		int lineType = 8;
		int pathline_thickness = 2;

		for(auto it = path.begin(); it != path.end()-1; it++)
		{
			// consecutive cells
			auto cell1 = (*it)->bundled_data_;
			auto cell2 = (*(it+1))->bundled_data_;

			// center coordinates
			x1 = cell1->location_.x;
			y1 = cell1->location_.y;

			x2 = cell2->location_.x;
			y2 = cell2->location_.y;

			cv::line( dst,
					cv::Point(x1,y1),
					cv::Point(x2,y2),
					//Scalar( 237, 149, 100 ),
					cv::Scalar( 255, 153, 51 ),
					pathline_thickness,
					lineType);
		}
	}

};

}

#endif /* SRC_VISUALIZER_GRAPH_VIS_H_ */
