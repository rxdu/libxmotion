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
#include "graph_vis/graph_vis.h"

using namespace srcl_ctrl;
using namespace cv;

MapViewer::MapViewer():
	disp_once(false),
	cell_decomp_method_(CellDecompMethod::SQUARE_GRID),
	qtree_depth_(6)
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

	return true;
}

void MapViewer::SaveResultToFile(std::string file_name)
{
	Mat image_to_save;

	map_image_.copyTo(image_to_save);

	if(!file_name.empty()) {
		imwrite(file_name, image_to_save);
	}
}

Mat MapViewer::DecomposeWorkspace(DecomposeConfig config)
{
    Mat vis_img;

    std::cout << "this function called" << std::endl;

    if(config.method == CellDecompMethod::SQUARE_GRID)
    {
    	std::tuple<std::shared_ptr<SquareGrid> , Mat> sg_map = SGridBuilder::BuildSquareGridMap(raw_image_, 32);
        std::shared_ptr<SquareGrid> sgrid = std::get<0>(sg_map);
        map_image_ = std::get<1>(sg_map);

        // build graph from square grid
        std::shared_ptr<Graph_t<SquareCell*>> sgrid_graph = GraphBuilder::BuildFromSquareGrid(sgrid, true);

        GraphVis vis;
        vis.VisSquareGrid(*sgrid, map_image_, vis_img);

        /*** put the graph on top of the square grid ***/
        vis.VisSquareGridGraph(*sgrid_graph, vis_img, vis_img, true);
        map_image_ = vis_img;

        std::cout << "decomposed using square grid" << std::endl;
    }
    else if(config.method == CellDecompMethod::QUAD_TREE)
    {
    	std::tuple<std::shared_ptr<QuadTree> , Mat> qt_map = QTreeBuilder::BuildQuadTreeMap(raw_image_, config.qtree_depth);
        std::shared_ptr<QuadTree> qtree = std::get<0>(qt_map);
        map_image_ = std::get<1>(qt_map);

        // build graph from quad tree
        std::shared_ptr<Graph_t<QuadTreeNode*>> qtree_graph = GraphBuilder::BuildFromQuadTree(qtree);

        GraphVis vis;
        vis.VisQuadTree(*qtree, map_image_, vis_img, TreeVisType::ALL_SPACE);
        vis.VisQTreeGraph(*qtree_graph, vis_img, vis_img, true, false);

        map_image_ = vis_img;

        std::cout << "decomposed using quadtree" << std::endl;
    }

    return vis_img;
}
