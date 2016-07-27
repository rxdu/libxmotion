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
//	Mat image_to_save;
//
//	if(ui->rbUseQTree->isChecked())
//	{
//		if(!qtree_map_.empty())
//		{
//			qtree_map_.copyTo(image_to_save);
//		}
//		else
//		{
//			ui->statusBar->showMessage(tr("No avaialable map data to save."));
//			return;
//		}
//	}
//	else if(ui->rbUseSGrid->isChecked())
//	{
//		if(!sgrid_map_.empty())
//		{
//			sgrid_map_.copyTo(image_to_save);;
//		}
//		else
//		{
//			ui->statusBar->showMessage(tr("No avaialable map data to save."));
//			return;
//		}
//	}
//
//	QString new_filename = QFileDialog::getSaveFileName(this,
//			tr("Save Map to File"),"/home/rdu/Workspace/srcl_robot_suite/srcl_ctrl/planning/data",tr("Map Images (*.png *.jpg)"));
//
//	if(!new_filename.contains(".jpg",Qt::CaseSensitivity::CaseInsensitive) &&
//			!new_filename.contains(".png",Qt::CaseSensitivity::CaseInsensitive))
//	{
//		new_filename.append(".jpg");
//	}
//
//	if(!new_filename.isEmpty()) {
//		imwrite(new_filename.toStdString(), image_to_save);
//	}
}

Mat MapViewer::DecomposeWorkspace(DecomposeConfig config)
{
    Mat vis_img;

    std::cout << "this function called" << std::endl;

    if(config.method == CellDecompMethod::SQUARE_GRID)
    {
        std::tuple<std::shared_ptr<SquareGrid> , Mat> sg_map;

        sg_map = SGridBuilder::BuildSquareGridMap(raw_image_, 32);
        sgrid_ = std::get<0>(sg_map);
        sgrid_map_ = std::get<1>(sg_map);

        // build graph from square grid
        sgrid_graph_ = GraphBuilder::BuildFromSquareGrid(sgrid_, true);

        GraphVis vis;

        if (sgrid_map_.empty())
            vis.VisSquareGrid(*sgrid_, vis_img);
        else
            vis.VisSquareGrid(*sgrid_, sgrid_map_, vis_img);

        /*** put the graph on top of the square grid ***/
        vis.VisSquareGridGraph(*sgrid_graph_, vis_img, vis_img, true);
        sgrid_map_ = vis_img;

        std::cout << "decomposed using square grid" << std::endl;
    }
    else if(config.method == CellDecompMethod::QUAD_TREE)
    {
        std::tuple<std::shared_ptr<QuadTree> , Mat> qt_map;

        qt_map = QTreeBuilder::BuildQuadTreeMap(raw_image_, config.qtree_depth);
        qtree_ = std::get<0>(qt_map);
        qtree_map_ = std::get<1>(qt_map);

        // build graph from square grid
        qtree_graph_ = GraphBuilder::BuildFromQuadTree(qtree_);

        GraphVis vis;
        vis.VisQuadTree(*qtree_, qtree_map_, vis_img, TreeVisType::ALL_SPACE);
        vis.VisQTreeGraph(*qtree_graph_, vis_img, vis_img, true, false);

        qtree_map_ = vis_img;

        std::cout << "decomposed using quadtree" << std::endl;
    }

    return vis_img;
}
