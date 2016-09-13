/*
 * test_combiner.cpp
 *
 *  Created on: Sep 12, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_PATH_REPAIR_TEST_COMBINER_CPP_
#define PLANNING_SRC_PATH_REPAIR_TEST_COMBINER_CPP_

#include <memory>
#include <ctime>

// octomap
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/comm.hpp"

// opencv
#include "opencv2/opencv.hpp"

#include "path_repair/graph_combiner.h"
#include "geometry/square_grid/square_grid.h"
#include "geometry/cube_array/cube_array.h"
#include "geometry/graph_builder.h"
#include "geometry/sgrid_builder.h"
#include "geometry/cube_array_builder.h"
#include "map/map_utils.h"

using namespace srcl_ctrl;
using namespace octomap;
using namespace cv;

int main(int argc, char* argv[])
{
	// read octomap data
	std::shared_ptr<octomap::OcTree> tree = std::make_shared<octomap::OcTree>(0.1);
	std::string tree_path = "/home/rdu/Workspace/srcl_rtk/srcl_ctrl/pc/planning/data/experiments/set2/octree_set2.bt";
	tree->readBinary(tree_path);

	// read 2d map data
	Mat input_image;
	std::string image_path = "/home/rdu/Workspace/srcl_rtk/srcl_ctrl/pc/planning/data/experiments/set1/map_path_repair.png";
	MapUtils::ReadImageFromFile(image_path, input_image);
	Map_t<SquareGrid> sgrid_map = SGridBuilder::BuildSquareGridMap(input_image, 32);
	sgrid_map.info.SetWorldSize(5.0, 5.0);
	sgrid_map.info.origin_offset_x = 2.5;
	sgrid_map.info.origin_offset_y = 2.5;

	std::shared_ptr<Graph_t<SquareCell*>> map_graph = GraphBuilder::BuildFromSquareGrid(sgrid_map.data_model,true);

	std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctree(tree);
	std::shared_ptr<Graph<CubeCell&>> cubegraph = GraphBuilder::BuildFromCubeArray(cubearray);

	////////////////////////////////////////////////////////////////////////////////////////////

	// combine graphs
	GraphCombiner<SquareCell*, SquareGrid> combiner;
	combiner.UpdateFlightHeight(Position3Dd(-1.8,0.6,0.8), Eigen::Quaterniond(0.923868 , 3.68874e-05 , 9.55242e-06 , -0.382712));
	combiner.SetBaseGraph(map_graph, sgrid_map.data_model, sgrid_map.data_model->cells_.size(), sgrid_map.info);

	clock_t		exec_time;
	exec_time = clock();
	combiner.CombineBaseWithCubeArrayGraph(cubearray, cubegraph);
	exec_time = clock() - exec_time;
	std::cout << "Graph construction finished in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

	exec_time = clock();
	combiner.CombineBaseWithCubeArrayGraph(cubearray, cubegraph);
	exec_time = clock() - exec_time;
	std::cout << "Graph construction 2 finished in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

	// search in combined graph
	uint64_t geo_start_id_astar = map_graph->GetVertexFromID(844)->bundled_data_->geo_mark_id_;
	uint64_t geo_goal_id_astar = map_graph->GetVertexFromID(187)->bundled_data_->geo_mark_id_;

	exec_time = clock();
	auto comb_path = combiner.combined_graph_.AStarSearch(geo_start_id_astar, geo_goal_id_astar);
	exec_time = clock() - exec_time;
	std::cout << "Search finished in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

	std::vector<Position3Dd> comb_path_pos;
	for(auto& wp:comb_path)
		comb_path_pos.push_back(wp->bundled_data_.position);

	////////////////////////////////////////////////////////////////////////////////////////////
	// send data for visualization
	std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

	if(!lcm->good())
	{
		std::cout << "ERROR: Failed to initialize LCM." << std::endl;
		return -1;
	}

	srcl_msgs::Graph_t graph_msg3;

	graph_msg3.vertex_num = combiner.combined_graph_.GetGraphVertices().size();
	for(auto& vtx : combiner.combined_graph_.GetGraphVertices())
	{
		srcl_msgs::Vertex_t vertex;
		vertex.id = vtx->vertex_id_;

		vertex.position[0] = vtx->bundled_data_.position.x;
		vertex.position[1] = vtx->bundled_data_.position.y;
		vertex.position[2] = vtx->bundled_data_.position.z;

		graph_msg3.vertices.push_back(vertex);
	}

	graph_msg3.edge_num = combiner.combined_graph_.GetGraphUndirectedEdges().size();
	for(auto& eg : combiner.combined_graph_.GetGraphUndirectedEdges())
	{
		srcl_msgs::Edge_t edge;
		edge.id_start = eg.src_->vertex_id_;
		edge.id_end = eg.dst_->vertex_id_;

		graph_msg3.edges.push_back(edge);
	}

	lcm->publish("quad/cube_graph", &graph_msg3);

	srcl_msgs::Path_t path_msg;

	path_msg.waypoint_num = comb_path.size();
	for(auto& wp : comb_path_pos)
	{
		srcl_msgs::WayPoint_t waypoint;
		waypoint.positions[0] = wp.x;
		waypoint.positions[1] = wp.y;
		waypoint.positions[2] = wp.z;

		path_msg.waypoints.push_back(waypoint);
	}

	lcm->publish("quad/geo_mark_graph_path", &path_msg);

	return 0;
}

#endif /* PLANNING_SRC_PATH_REPAIR_TEST_COMBINER_CPP_ */
