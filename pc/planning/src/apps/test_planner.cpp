/*
 * test_planner.cpp
 *
 *  Created on: Aug 4, 2016
 *      Author: rdu
 */

#include <vector>
#include <memory>

#include "vis/rrt_vis.h"

#include "map/graph_builder.h"
#include "map/sgrid_builder.h"
#include "map/map_utils.h"
#include "map/map_config.h"
#include "map/map_info.h"
#include "planner/rrts_planner.h"
#include "planner/quad_planner.h"
#include "common/planning_types.h"
#include "vis/vis_utils.h"
#include "vis/graph_vis.h"

using namespace srcl_ctrl;
using namespace cv;

int main(int argc, char** argv)
{
	std::string image_dir = "/home/rdu/Workspace/srcl_robot_suite/srcl_ctrl/pc/planning/data/example.png";

//	Mat vis_img;
//	MapInfo map_info;
//
//	map_info.map_size_x = 1000;
//	map_info.map_size_y = 1000;
//	map_info.SetWorldSize(1.0, 1.0);
//	ImageUtils::CreateOccupancyMapForRRT(map_info.map_size_x,map_info.map_size_y, vis_img);
//	BoundingBox bbox;
//	bbox.x.min = 300;
//	bbox.x.max = 600;
//	bbox.y.min = 300;
//	bbox.y.max = 600;
//	VisUtils::FillRectangularArea(vis_img, bbox, cv::Scalar(0));

	QuadPlanner qplanner;
	MapConfig map_config;
	map_config.SetMapPath(image_dir);
	map_config.SetMapType(MapDataModel::SQUARE_GRID, 32);
	//map_config.SetMapType(MapDataModel::QUAD_TREE, 6);

	qplanner.ConfigGraphPlanner(map_config);

	if(qplanner.active_graph_planner_ == GraphPlannerType::NOT_SPECIFIED)
	{
		std::cout << "failed to init quad planner" << std::endl;
		return -1;
	}

	// test global graph planner
	Position2D start_pos, goal_pos;
	start_pos.x = 25;
	start_pos.y = 183;
	goal_pos.x = 972;
	goal_pos.y = 467;
	qplanner.SetStartMapPosition(start_pos);
	qplanner.SetGoalMapPosition(goal_pos);
	auto global_solution = qplanner.SearchForGlobalPath();

	// test local rrts planner
	qplanner.SetRealWorldSize(10.0, 5.0);

	Position2Dd start,goal;
	Position2D start_ori, goal_ori;
	MapInfo info = qplanner.GetActiveMapInfo();
	start_ori = MapUtils::CoordinatesFromPaddedToOriginal(start_pos, info);
	goal_ori = MapUtils::CoordinatesFromPaddedToOriginal(goal_pos, info);
	start = MapUtils::CoordinatesFromMapToWorld(start_ori, info);
	goal = MapUtils::CoordinatesFromMapToWorld(goal_ori, info);

	std::vector<Position2Dd> path;
	bool result = qplanner.local_planner_.SearchSolution(start, goal,10.5, path);

	/*--------------------- Visualization ----------------------*/
	if(!global_solution.empty()) {
		Mat input_image;
		MapUtils::ReadImageFromFile(image_dir, input_image);
		Map_t<SquareGrid> sgrid_map = SGridBuilder::BuildSquareGridMap(input_image, 32);
		std::shared_ptr<Graph_t<SquareCell*>> graph = GraphBuilder::BuildFromSquareGrid(sgrid_map.data_model,true);

		std::vector<Vertex_t<SquareCell*>*> traj;
		for(auto& wp:global_solution)
			traj.push_back(graph->GetVertexFromID(wp));

		Mat vis_img;
		GraphVis::VisSquareGrid(*sgrid_map.data_model, sgrid_map.padded_image, vis_img);
		GraphVis::VisSquareGridGraph(*graph, vis_img, vis_img, true);
		GraphVis::VisSquareGridPath(traj, vis_img, vis_img);

		// display visualization result
		namedWindow("Graph Plan", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
		imshow("Graph Plan", vis_img);
	}

	if(result) {
		Mat rrt_vis;
		if(result)
		{
			RRTVis::VisRRTPath(path, info, qplanner.GetActiveMap(), rrt_vis);
			//RRTVis::VisRRTPath(path, info, vis_img, rrt_vis);
			RRTVis::VisRRTGraph(*qplanner.local_planner_.rrts_vis_graph_, info, rrt_vis, rrt_vis);
		}

		namedWindow("RRTS Plan", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
		imshow("RRTS Plan", rrt_vis);
	}

	waitKey(0);
}



