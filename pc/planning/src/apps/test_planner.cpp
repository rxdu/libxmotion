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

	QuadPlanner qplanner;
	MapConfig map_config;
	map_config.SetMapPath(image_dir);
	map_config.SetMapType(MapDataModel::SQUARE_GRID, 32);
	//map_config.SetMapType(MapDataModel::QUAD_TREE, 6);

	qplanner.ConfigGraphPlanner(map_config);
	qplanner.SetRealWorldSize(10.0, 5.0);

	if(qplanner.active_graph_planner_ == GraphPlannerType::NOT_SPECIFIED)
	{
		std::cout << "failed to init quad planner" << std::endl;
		return -1;
	}

	// test global graph planner
	Position2D start_pos, goal_pos;
//	start_pos.x = 317;
//	start_pos.y = 304;
//	goal_pos.x = 638;
//	goal_pos.y = 493;
	start_pos.x = 18;
	start_pos.y = 177;
	goal_pos.x = 968;
	goal_pos.y = 840;
	qplanner.SetStartMapPosition(start_pos);
	qplanner.SetGoalMapPosition(goal_pos);
	auto global_solution = qplanner.SearchForGlobalPathID();

	// test local rrts planner
	Position2Dd start,goal;
	Position2D start_ori, goal_ori;
	MapInfo info = qplanner.GetActiveMapInfo();
	start_ori = MapUtils::CoordinatesFromPaddedToOriginal(start_pos, info);
	goal_ori = MapUtils::CoordinatesFromPaddedToOriginal(goal_pos, info);
	start = MapUtils::CoordinatesFromMapToMapWorld(start_ori, info);
	goal = MapUtils::CoordinatesFromMapToMapWorld(goal_ori, info);
	std::vector<Position2Dd> path;
	bool result = qplanner.SearchForLocalPath(start, goal,20, path);

	/*--------------------- Visualization ----------------------*/
	std::cout << "\n*********************************************************\n" << std::endl;
	Mat graph_vis;
	Mat rrt_vis;

	if(!global_solution.empty()) {
		Mat input_image;
		MapUtils::ReadImageFromFile(image_dir, input_image);
		Map_t<SquareGrid> sgrid_map = SGridBuilder::BuildSquareGridMap(input_image, 32);
		std::shared_ptr<Graph_t<SquareCell*>> graph = GraphBuilder::BuildFromSquareGrid(sgrid_map.data_model,true);

		std::vector<Vertex_t<SquareCell*>*> traj;
		for(auto& wp:global_solution)
			traj.push_back(graph->GetVertexFromID(wp));

		GraphVis::VisSquareGrid(*sgrid_map.data_model, sgrid_map.padded_image, graph_vis);
		GraphVis::VisSquareGridGraph(*graph, graph_vis, graph_vis, true);
		GraphVis::VisSquareGridPath(traj, graph_vis, graph_vis);

		//imwrite( "graph_vis.jpg", graph_vis);

		// display visualization result
		namedWindow("Graph Plan", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
		imshow("Graph Plan", graph_vis);
	}
	else
		std::cout << "graph solution not found" << std::endl;

	if(result) {
		RRTVis::VisRRTPath(path, info, qplanner.GetActiveMap(), rrt_vis);
		//RRTVis::VisRRTPath(path, info, graph_vis, rrt_vis);

		//imwrite( "rrt_vis_path.jpg", rrt_vis);

		RRTVis::VisRRTGraph(*qplanner.GetLocalPlannerVisGraph(), info, rrt_vis, rrt_vis);

		//imwrite( "rrt_vis_tree.jpg", rrt_vis);

		namedWindow("RRTS Plan", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
		imshow("RRTS Plan", rrt_vis);
	}
	else
		std::cout << "rrts solution not found" << std::endl;

	waitKey(0);
}



