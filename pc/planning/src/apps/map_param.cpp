/*
 * map_param.cpp
 *
 *  Created on: Nov 21, 2016
 *      Author: rdu
 */

// standard libaray
#include <iostream>
#include <vector>
#include <ctime>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "graph/graph.h"
#include "graph/astar.h"
#include "vis/graph_vis.h"
#include "map/image_utils.h"
#include "map/map_utils.h"
#include "planner/graph_planner.h"
#include "geometry/graph_builder.h"
#include "geometry/sgrid_builder.h"

using namespace cv;
using namespace srcl_ctrl;

Position2D ConvertGoalRefWorldPositionToMapPosition(Position2Dd pos, MapInfo info)
{
	std::cout << "\nposition in ref world: " << pos.x << " , " << pos.y << std::endl;

	Position2Dd mpos;
	mpos = MapUtils::CoordinatesFromRefWorldToMapWorld(pos, info);
	std::cout << "position in map world: " << mpos.x << " , " << mpos.y << std::endl;

	Position2D map_pos;
	map_pos = MapUtils::CoordinatesFromMapWorldToMap(mpos, info);
	std::cout << "position in map: " << map_pos.x << " , " << map_pos.y << std::endl;

	Position2D map_padded_pos;
	map_padded_pos = MapUtils::CoordinatesFromOriginalToPadded(map_pos, info);
	std::cout << "position in padded map: " << map_padded_pos.x << " , " << map_padded_pos.y << std::endl;

	return map_padded_pos;
}

int main(int argc, char** argv )
{
	bool show_padding = false;
	GraphPlanner<SquareGrid> sgrid_planner;

	/*** Config graph planner ***/
	std::string image_dir = "/home/rdu/Workspace/srcl_rtk/srcl_ctrl/pc/planning/data/map_testcase2.png";
	MapConfig map_config;

	map_config.SetMapPath(image_dir);
	map_config.SetMapType(MapDataModel::SQUARE_GRID, 16);
	map_config.SetOriginOffset(12.5, 10.0);

	sgrid_planner.UpdateMapConfig(map_config);
	sgrid_planner.map_.info.SetWorldSize(20.0, 25.0);

	/*** Search path in the graph ***/
	Position2Dd start_w(-11.0,8.5);
	Position2Dd goal_w(11.0, -8.5);

	//Position2D start_m = ConvertGoalRefWorldPositionToMapPosition(start_w, sgrid_planner.map_.info);
	Position2D start_m = MapUtils::CoordinatesFromRefWorldToMapPadded(start_w, sgrid_planner.map_.info);
	Position2D goal_m = MapUtils::CoordinatesFromRefWorldToMapPadded(goal_w, sgrid_planner.map_.info);

	auto start_vertex_id = sgrid_planner.map_.data_model->GetIDFromPosition(start_m.x, start_m.y);
	auto goal_vertex_id = sgrid_planner.map_.data_model->GetIDFromPosition(goal_m.x, goal_m.y);

	auto start_vertex = sgrid_planner.graph_->GetVertexFromID(start_vertex_id);
	auto goal_vertex = sgrid_planner.graph_->GetVertexFromID(goal_vertex_id);

	Path_t<SquareCell*> path;
	if(start_vertex == nullptr || goal_vertex == nullptr)
	{
		if(start_vertex == nullptr)
			std::cout << "Invalid start" << std::endl;

		if(goal_vertex == nullptr)
			std::cout << "Invalid goal" << std::endl;
	}
	else
	{
		clock_t		exec_time;
		exec_time = clock();
		path = sgrid_planner.Search(start_vertex_id, goal_vertex_id);
		exec_time = clock() - exec_time;
		std::cout << "Searched in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;
	}

	/*** Visualize the map and graph ***/
	Mat vis_img;

	/*** Image Layouts: (map) -> square grid -> graph -> path ***/
	GraphVis::VisSquareGrid(*sgrid_planner.map_.data_model, sgrid_planner.map_.padded_image, vis_img);

	/*** put the graph on top of the square grid ***/
	GraphVis::VisSquareGridGraph(*sgrid_planner.graph_, vis_img, vis_img, true);
	/*** put the path on top of the graph ***/
	if(!path.empty())
		GraphVis::VisSquareGridPath(path, vis_img, vis_img);

	if(!show_padding)
	{
		Range rngx(0 + sgrid_planner.map_.info.padded_left, vis_img.cols - sgrid_planner.map_.info.padded_right);
		Range rngy(0 + sgrid_planner.map_.info.padded_top, vis_img.rows - sgrid_planner.map_.info.padded_bottom);

		// Points and Size go (x,y); (width,height) ,- Mat has (row,col).
		vis_img = vis_img(rngy,rngx);
	}

	// display visualization result
	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", vis_img);

	waitKey(0);

//	imwrite( "new_map_path_cmp2.jpg", vis_result);

	return 0;
}



