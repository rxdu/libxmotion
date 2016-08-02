/*
 * map_viewer.cpp
 *
 *  Created on: Jul 26, 2016
 *      Author: rdu
 */

#include "map_viewer.h"

#include "map/graph_builder.h"
#include "map/qtree_builder.h"
#include "map/sgrid_builder.h"

using namespace srcl_ctrl;
using namespace cv;

MapViewer::MapViewer():
	image_updated_(false),
	squarecell_size_(0),
	qtree_depth_(0),
	active_decompose_(CellDecompMethod::NOT_SPECIFIED)
{

}

MapViewer::~MapViewer()
{

}

bool MapViewer::ReadMapFromFile(std::string file_name)
{
	if(!file_name.empty()) {
		// read map image
		raw_image_ = imread(file_name, IMREAD_GRAYSCALE);

		if (!raw_image_.data) {
			printf("No image data \n");
			return false;
		}
	}
	else
		return false;

	// reset flags if successfully read new image
	image_updated_ = true;
	qtree_depth_ = 0;
	squarecell_size_ = 0;
	active_decompose_ = CellDecompMethod::NOT_SPECIFIED;

	return true;
}

void MapViewer::SaveResultToFile(std::string file_name)
{
	Mat image_to_save;

	displayed_image_.copyTo(image_to_save);

	if(!file_name.empty()) {
		imwrite(file_name, image_to_save);
	}
}

cv::Mat MapViewer::DecomposeWorkspace(DecomposeConfig config, MapInfo &info)
{
    Mat vis_img;

    //std::cout << "trying to decompose workspace" << std::endl;

    bool config_changed = false;
    if((squarecell_size_ != config.square_cell_size) ||
        		qtree_depth_ != config.qtree_depth)
    	config_changed = true;

    if(image_updated_ || config_changed) {
    	image_updated_ = false;
    	squarecell_size_ = config.square_cell_size;
    	qtree_depth_ = config.qtree_depth;

    	// build square grid map from image
    	sg_map_ = SGridBuilder::BuildSquareGridMap(raw_image_, squarecell_size_);

    	// build graph from square grid
    	sgrid_graph_ = GraphBuilder::BuildFromSquareGrid(sg_map_.data_model, true);

    	// build quad tree map from image
    	qt_map_ = QTreeBuilder::BuildQuadTreeMap(raw_image_, qtree_depth_);

    	// build graph from quad tree
    	qtree_graph_ = GraphBuilder::BuildFromQuadTree(qt_map_.data_model);

    	std::cout << "internal data structure updated" << std::endl;
    }

    if(config.method == CellDecompMethod::SQUARE_GRID)
    {
    	info = sg_map_.info;
    	active_decompose_ = CellDecompMethod::SQUARE_GRID;

    	graph_vis_.VisSquareGrid(*sg_map_.data_model, sg_map_.padded_image, vis_img);

    	/*** put the graph on top of the square grid ***/
    	graph_vis_.VisSquareGridGraph(*sgrid_graph_, vis_img, vis_img, true);

    	displayed_image_ = vis_img;

    	//std::cout << "decomposed using square grid" << std::endl;
    }
    else if(config.method == CellDecompMethod::QUAD_TREE)
    {
    	info = qt_map_.info;
    	active_decompose_ = CellDecompMethod::QUAD_TREE;

    	graph_vis_.VisQuadTree(*qt_map_.data_model, qt_map_.padded_image, vis_img, TreeVisType::ALL_SPACE);
    	graph_vis_.VisQTreeGraph(*qtree_graph_, vis_img, vis_img, true, false);

    	displayed_image_ = vis_img;

    	//std::cout << "decomposed using quadtree" << std::endl;
    }

    return vis_img;
}

cv::Mat MapViewer::HighlightSelectedNode(uint32_t x, uint32_t y, bool& updated)
{
	cv::Mat vis_img;
	uint64_t id;

	updated = false;

	if(active_decompose_ == CellDecompMethod::SQUARE_GRID)
	{
		id = sg_map_.data_model->GetIDFromPosition(x, y);

		auto vtx = sgrid_graph_->GetVertexFromID(id);

//		std::cout << " ---- " << std::endl;
//		std::cout << "cell size: " << sg_map_.data_model->cell_size_ << std::endl;
//		std::cout << "selected id: " << id << " at position: " << x << " , " << y << std::endl;;

		graph_vis_.VisSquareGrid(*sg_map_.data_model, sg_map_.padded_image, vis_img);
		graph_vis_.VisSquareGridGraph(*sgrid_graph_, vis_img, vis_img, true);

		if(vtx != nullptr)
		{
			auto node = vtx->bundled_data_;

			if(node->occu_ == OccupancyType::FREE || node->occu_ == OccupancyType::INTERESTED) {

				Range rngx(sg_map_.data_model->cells_[id]->bbox_.x.min, sg_map_.data_model->cells_[id]->bbox_.x.max);
				Range rngy(sg_map_.data_model->cells_[id]->bbox_.y.min, sg_map_.data_model->cells_[id]->bbox_.y.max);

				vis_img(rngy,rngx) = Scalar(0,255,255);

				updated = true;
			}

			displayed_image_ = vis_img;
		}
		else
			displayed_image_ = vis_img;

	}
	else if(active_decompose_ == CellDecompMethod::QUAD_TREE)
	{
		id = qt_map_.data_model->GetIDFromPosition(x, y);

		auto vtx = qtree_graph_->GetVertexFromID(id);

		graph_vis_.VisQuadTree(*qt_map_.data_model, qt_map_.padded_image, vis_img, TreeVisType::ALL_SPACE);
		graph_vis_.VisQTreeGraph(*qtree_graph_, vis_img, vis_img, true, false);

		if(id == 0 || vtx == nullptr)
		{
			displayed_image_ = vis_img;
		}
		else
		{
			auto node = vtx->bundled_data_;

			Range rngx(node->bounding_box_.x.min, node->bounding_box_.x.max);
			Range rngy(node->bounding_box_.y.min, node->bounding_box_.y.max);

			vis_img(rngy,rngx) = Scalar(0,255,255);

			displayed_image_ = vis_img;

			updated = true;
		}
	}

	std::cout << "clicked node id: " << id << std::endl;

	return vis_img;
}

cv::Mat MapViewer::DisplayTrajectory(std::vector<uint64_t>& traj)
{
	cv::Mat vis_img;
	uint64_t id;

	if(active_decompose_ == CellDecompMethod::SQUARE_GRID)
	{
		std::vector<Vertex_t<SquareCell*>*> traj_vertices;
		for(auto& wp_id : traj)
			traj_vertices.push_back(sgrid_graph_->GetVertexFromID(wp_id));

		graph_vis_.VisSquareGrid(*sg_map_.data_model, sg_map_.padded_image, vis_img);
		graph_vis_.VisSquareGridGraph(*sgrid_graph_, vis_img, vis_img, true);
		graph_vis_.VisSquareGridPath(traj_vertices, vis_img, vis_img);

		displayed_image_ = vis_img;

	}
	else if(active_decompose_ == CellDecompMethod::QUAD_TREE)
	{
		std::vector<Vertex_t<QuadTreeNode*>*> traj_vertices;
		for(auto& wp_id : traj)
			traj_vertices.push_back(qtree_graph_->GetVertexFromID(wp_id));

		graph_vis_.VisQuadTree(*qt_map_.data_model, qt_map_.padded_image, vis_img, TreeVisType::ALL_SPACE);
		graph_vis_.VisQTreeGraph(*qtree_graph_, vis_img, vis_img, true, false);
		graph_vis_.VisQTreeGraphPath(traj_vertices, vis_img, vis_img);

		displayed_image_ = vis_img;
	}

	std::cout << "clicked node id: " << id << std::endl;

	return vis_img;
}
