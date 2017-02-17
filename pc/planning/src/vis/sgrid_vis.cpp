/*
 * sgrid_vis.cpp
 *
 *  Created on: Feb 9, 2017
 *      Author: rdu
 */


#include "vis/sgrid_vis.h"
#include "vis/vis_utils.h"

using namespace srcl_ctrl;
using namespace srcl_ctrl::VisUtils;
using namespace cv;

void Vis::VisSquareGrid(const SquareGrid& grid, cv::OutputArray _dst)
{
	_dst.create(Size(grid.col_size_*grid.cell_size_, grid.row_size_*grid.cell_size_), CV_8UC3);
	Mat dst = _dst.getMat();
	dst = bk_color_;

	// fill cell color
	for(auto itc = grid.cells_.begin(); itc != grid.cells_.end(); itc++)
	{
		if((*itc).second->occu_ == OccupancyType::OCCUPIED)
			VisUtils::FillRectangularArea(dst, (*itc).second->bbox_, obs_color_);
		else if((*itc).second->occu_ == OccupancyType::INTERESTED)
			VisUtils::FillRectangularArea(dst, (*itc).second->bbox_, aoi_color_);

		auto cell = (*itc);
		uint64_t x,y;
		x = cell.second->bbox_.x.min + (cell.second->bbox_.x.max - cell.second->bbox_.x.min)/2;
		x = x + (cell.second->bbox_.x.max - cell.second->bbox_.x.min)/6;
		y = cell.second->bbox_.y.min + (cell.second->bbox_.y.max - cell.second->bbox_.y.min)/2;
		y = y + (cell.second->bbox_.y.max - cell.second->bbox_.y.min)*3/7;

		std::string id = std::to_string(cell.second->data_id_);

		putText(dst, id ,Point(x,y), CV_FONT_NORMAL, 0.5, Scalar(0,0,0),1,1);
	}

	// draw grid lines
	line(dst, Point(0,0),Point(0,grid.row_size_*grid.cell_size_-1),ln_color_, 1);
	for(int i = 1; i <= grid.col_size_; i++){
		line(dst, Point(i*grid.cell_size_-1,0),Point(i*grid.cell_size_-1,grid.row_size_*grid.cell_size_-1),ln_color_, 1);
	}

	line(dst, Point(0,0),Point(grid.col_size_*grid.cell_size_-1,0),ln_color_, 1);
	for(int i = 1; i <= grid.row_size_; i++){
		line(dst, Point(0,i*grid.cell_size_-1),Point(grid.col_size_*grid.cell_size_-1,i*grid.cell_size_-1),ln_color_, 1);
	}
}

void Vis::VisSquareGrid(const SquareGrid& grid, cv::InputArray _src, cv::OutputArray _dst)
{
	Mat src_img_color;
	cvtColor(_src, src_img_color, CV_GRAY2BGR);
	_dst.create(src_img_color.size(), src_img_color.type());
	Mat dst = _dst.getMat();

	std::vector<SquareCell*> cells;
	for(auto it = grid.cells_.begin(); it != grid.cells_.end(); it++)
	{
		cells.push_back((*it).second);
	}

	for(auto itc = cells.begin(); itc != cells.end(); itc++)
	{
		Point top_left((*itc)->bbox_.x.min, (*itc)->bbox_.y.min);
		Point top_right((*itc)->bbox_.x.max,(*itc)->bbox_.y.min);
		Point bot_left((*itc)->bbox_.x.min,(*itc)->bbox_.y.max);
		Point bot_right((*itc)->bbox_.x.max,(*itc)->bbox_.y.max);

		line(src_img_color, top_left, top_right, Scalar(0,255,0));
		line(src_img_color, top_right, bot_right, Scalar(0,255,0));
		line(src_img_color, bot_right, bot_left, Scalar(0,255,0));
		line(src_img_color, bot_left, top_left, Scalar(0,255,0));

		/*auto cell = (*itc);
		uint64_t x,y;
		x = cell->bbox_.x.min + (cell->bbox_.x.max - cell->bbox_.x.min)/2;
		x = x + (cell->bbox_.x.max - cell->bbox_.x.min)/6;
		y = cell->bbox_.y.min + (cell->bbox_.y.max - cell->bbox_.y.min)/2;
		y = y + (cell->bbox_.y.max - cell->bbox_.y.min)*3/7;
		std::string id = std::to_string(cell->data_id_);

		putText(src_img_color, id ,Point(x,y), CV_FONT_NORMAL, 0.5, Scalar(0,0,0),1,1);
		*/
	}

	src_img_color.copyTo(dst);
}

void Vis::VisSquareGridNavField(const SquareGrid& grid, const NavField<SquareCell*>& nav_field, cv::InputArray _src, cv::OutputArray _dst, bool show_id)
{
	_dst.create(Size(grid.col_size_*grid.cell_size_, grid.row_size_*grid.cell_size_), CV_8UC3);
	Mat dst = _dst.getMat();
	dst = bk_color_;

	// draw potential filed
//	int thickness = 2;
//	int lineType = 8;
//
//	for(int i = 1; i <= 30; ++i) {
//		circle( dst,
//				Point(nav_field.field_center_->bundled_data_->location_.x, nav_field.field_center_->bundled_data_->location_.y),
//				95*i,
//				Scalar(rand() % 255,rand() % 255,rand() % 255),
//				thickness,
//				lineType );
//	}

	// fill cell color
	for(auto itc = grid.cells_.begin(); itc != grid.cells_.end(); itc++)
	{
		if((*itc).second->occu_ == OccupancyType::OCCUPIED)
			VisUtils::FillRectangularArea(dst, (*itc).second->bbox_, obs_color_);
		//		else if((*itc).second->occu_ == OccupancyType::INTERESTED)
		//			VisUtils::FillRectangularArea(dst, (*itc).second->bbox_, aoi_color_);

		if(show_id) {
			auto cell = (*itc);
			uint64_t x,y;
			x = cell.second->bbox_.x.min + (cell.second->bbox_.x.max - cell.second->bbox_.x.min)/2;
			x = x + (cell.second->bbox_.x.max - cell.second->bbox_.x.min)/6;
			y = cell.second->bbox_.y.min + (cell.second->bbox_.y.max - cell.second->bbox_.y.min)/2;
			y = y + (cell.second->bbox_.y.max - cell.second->bbox_.y.min)*3/7;

			std::string id = std::to_string(cell.second->data_id_);

			putText(dst, id ,Point(x,y), CV_FONT_NORMAL, 0.5, Scalar(0,0,0),1,1);
		}
	}

	// draw grid lines
	line(dst, Point(0,0),Point(0,grid.row_size_*grid.cell_size_-1),ln_color_, 1);
	for(int i = 1; i <= grid.col_size_; i++){
		line(dst, Point(i*grid.cell_size_-1,0),Point(i*grid.cell_size_-1,grid.row_size_*grid.cell_size_-1),ln_color_, 1);
	}

	line(dst, Point(0,0),Point(grid.col_size_*grid.cell_size_-1,0),ln_color_, 1);
	for(int i = 1; i <= grid.row_size_; i++){
		line(dst, Point(0,i*grid.cell_size_-1),Point(grid.col_size_*grid.cell_size_-1,i*grid.cell_size_-1),ln_color_, 1);
	}

	// draw all vertices
	std::vector<Vertex<SquareCell*>*> vertices;
	vertices = nav_field.field_graph_->GetGraphVertices();
	for(auto itv = vertices.begin(); itv != vertices.end(); itv++)
	{
		cv::Point center((*itv)->bundled_data_->location_.x, (*itv)->bundled_data_->location_.y);
		VisUtils::DrawPoint(dst, center);

		// current vertex center coordinate
		uint64_t x1,y1,x2,y2;
		x1 = (*itv)->bundled_data_->location_.x;
		y1 = (*itv)->bundled_data_->location_.y;

//		if(show_id) {
//			if((*itv)->bundled_data_->data_id_ % 2 == 0)
//			{
//				std::string id = std::to_string((*itv)->bundled_data_->data_id_);
//				putText(dst, id ,Point(x1,y1), CV_FONT_NORMAL, 0.5, Scalar(204,204,102),1,1);
//			}
//		}

		// display potential value
		putText(dst, std::to_string((int)(*itv)->potential_) ,Point(x1,y1), CV_FONT_NORMAL, 0.5, Scalar(204,204,102),1,1);

//		if((*itv)->potential_ > start_vtx->potential_)
//			VisUtils::FillRectangularArea(dst, (*itv)->bundled_data_->bbox_, Scalar(211,211,211));
	}

	// draw all edges
	auto edges = nav_field.field_graph_->GetGraphUndirectedEdges();
	for(auto it = edges.begin(); it != edges.end(); it++)
	{
		uint64_t x1,y1,x2,y2;
		x1 = (*it).src_->bundled_data_->location_.x;
		y1 = (*it).src_->bundled_data_->location_.y;
		x2 = (*it).dst_->bundled_data_->location_.x;
		y2 = (*it).dst_->bundled_data_->location_.y;

		VisUtils::DrawLine(dst, Point(x1,y1), Point(x2,y2));
	}
}

void Vis::VisSquareGridLocalNavField(const SquareGrid& grid, const NavField<SquareCell*>& nav_field, Vertex_t<SquareCell*>* center_vtx, cv::InputArray _src, cv::OutputArray _dst, uint16_t sensor_range)
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

	auto nbs = const_cast<SquareGrid&>(grid).GetNeighboursWithinRange(center_vtx->bundled_data_->data_id_,sensor_range);
	for(auto& n : nbs) {
		auto vtx = nav_field.field_graph_->GetVertexFromID(n->data_id_);
		if(vtx == nullptr)
			continue;

		// calc shortcut rewards
		double dist = 0;
		Position2D start = center_vtx->bundled_data_->index_;
		Position2D goal = vtx->bundled_data_->index_;

		uint32_t x_error, y_error;

		if(start.x > goal.x)
			x_error = start.x - goal.x;
		else
			x_error = goal.x - start.x;

		if(start.y > goal.y)
			y_error = start.y - goal.y;
		else
			y_error = goal.y - start.y;

		uint32_t diag_steps = 0;
		uint32_t straight_steps = 0;

		if(x_error > y_error) {
			diag_steps = y_error;
			straight_steps = x_error - y_error;
		}
		else {
			diag_steps = x_error;
			straight_steps = y_error - x_error;
		}

		dist = diag_steps * std::sqrt(2) * grid.cell_size_ + straight_steps * grid.cell_size_;

		double rewards = center_vtx->potential_-vtx->potential_ - dist;

		if(rewards <= 0)
			VisUtils::FillRectangularArea(dst, vtx->bundled_data_->bbox_, Scalar(211,211,211));
		else {
			VisUtils::FillRectangularArea(dst, vtx->bundled_data_->bbox_, aoi_color_);
		}

		uint64_t x1,y1,x2,y2;
		x1 = vtx->bundled_data_->location_.x;
		y1 = vtx->bundled_data_->location_.y;

		// display potential value
		putText(dst, std::to_string((int)(rewards)) ,Point(x1,y1), CV_FONT_NORMAL, 0.5, Scalar(204,204,102),1,1);

	}
}

void Vis::VisSquareGridShortcutPotential(const NavField<SquareCell*>& nav_field, cv::InputArray _src, cv::OutputArray _dst)
{
	Mat src = _src.getMat();
	_dst.create(_src.size(), _src.type());
	Mat dst = _dst.getMat();
	src.copyTo(dst);

	// draw all vertices
	auto vertices = nav_field.field_graph_->GetGraphVertices();
	for(auto itv = vertices.begin(); itv != vertices.end(); itv++)
	{
		cv::Point center((*itv)->bundled_data_->location_.x, (*itv)->bundled_data_->location_.y);
		//DrawNodeCenter(center, dst);
		VisUtils::DrawPoint(dst, center);

		const double max_s = 0.9;
		HSVColor hsvc;
		hsvc.h = 40;
		hsvc.v = 1;
		hsvc.s = (*itv)->shortcut_rewards_/nav_field.max_rewards_ * max_s;
		if(hsvc.s > max_s)
			hsvc.s = max_s;
		RGBColor rgbc = VisUtils::HSV2RGB(hsvc);

		if((*itv)->shortcut_rewards_ > 1)
			VisUtils::FillRectangularArea(dst, (*itv)->bundled_data_->bbox_, Scalar(rgbc.b,rgbc.g,rgbc.r));//aoi_color_);

		// current vertex center coordinate
		uint64_t x1,y1,x2,y2;
		x1 = (*itv)->bundled_data_->location_.x;
		y1 = (*itv)->bundled_data_->location_.y;

		std::string id = std::to_string((int)(*itv)->shortcut_rewards_);
		putText(dst, id ,Point(x1,y1), CV_FONT_NORMAL, 0.5, Scalar(204,204,102),1,1);

		// draw all edges
		auto edges = nav_field.field_graph_->GetGraphUndirectedEdges();
		for(auto it = edges.begin(); it != edges.end(); it++)
		{
			uint64_t x1,y1,x2,y2;
			x1 = (*it).src_->bundled_data_->location_.x;
			y1 = (*it).src_->bundled_data_->location_.y;
			x2 = (*it).dst_->bundled_data_->location_.x;
			y2 = (*it).dst_->bundled_data_->location_.y;

			//DrawEdge(Point(x1,y1), Point(x2,y2), dst);
			VisUtils::DrawLine(dst, Point(x1,y1), Point(x2,y2));
		}
	}
}

