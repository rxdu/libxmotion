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
#include "map/map_type.h"

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
    Mat vis_img, temp_img;

    std::cout << "trying to decompose workspace" << std::endl;

    if(config.method == CellDecompMethod::SQUARE_GRID)
    {
    	Map_t<SquareGrid> sg_map;

    	sg_map = SGridBuilder::BuildSquareGridMap(raw_image_, 32);

        // build graph from square grid
        std::shared_ptr<Graph_t<SquareCell*>> sgrid_graph = GraphBuilder::BuildFromSquareGrid(sg_map.data_model, true);

        GraphVis vis;
        vis.VisSquareGrid(*sg_map.data_model, sg_map.padded_image, temp_img);

        /*** put the graph on top of the square grid ***/
        vis.VisSquareGridGraph(*sgrid_graph, temp_img, temp_img, true);
        map_image_ = temp_img;

        if(config.show_padded_area)
        	vis_img = map_image_;
        else
        {
        	Range rngx(0 + sg_map.info.padded_left, temp_img.cols - sg_map.info.padded_right);
        	Range rngy(0 + sg_map.info.padded_top, temp_img.rows - sg_map.info.padded_bottom);

        	// Points and Size go (x,y); (width,height) ,- Mat has (row,col).
        	vis_img = map_image_(rngy,rngx);
        }

        std::cout << "decomposed using square grid" << std::endl;
    }
    else if(config.method == CellDecompMethod::QUAD_TREE)
    {
    	Map_t<QuadTree> qt_map = QTreeBuilder::BuildQuadTreeMap(raw_image_, config.qtree_depth);

        // build graph from quad tree
        std::shared_ptr<Graph_t<QuadTreeNode*>> qtree_graph = GraphBuilder::BuildFromQuadTree(qt_map.data_model);

        GraphVis vis;
        vis.VisQuadTree(*qt_map.data_model, qt_map.padded_image, temp_img, TreeVisType::ALL_SPACE);
        vis.VisQTreeGraph(*qtree_graph, temp_img, temp_img, true, false);

        map_image_ = temp_img;

        if(config.show_padded_area)
        	vis_img = map_image_;
        else
        {
        	Range rngx(0 + qt_map.info.padded_left, temp_img.cols - qt_map.info.padded_right);
        	Range rngy(0 + qt_map.info.padded_top, temp_img.rows - qt_map.info.padded_bottom);

        	vis_img = map_image_(rngy,rngx);
        }

        std::cout << "decomposed using quadtree" << std::endl;
    }

    return vis_img;
}
