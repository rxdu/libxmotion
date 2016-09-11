/*
 * graph_combiner.cpp
 *
 *  Created on: Sep 9, 2016
 *      Author: rdu
 */

#include <memory>
#include <iostream>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/comm.hpp"

// octomap
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// opencv
#include "opencv2/opencv.hpp"

// user
#include "map/map_type.h"
#include "map/map_utils.h"
#include "graph/graph.h"
#include "geometry/graph_builder.h"
#include "geometry/sgrid_builder.h"
#include "geometry/cube_array/cube_array.h"
#include "geometry/cube_array_builder.h"
#include "vis/graph_vis.h"
#include "path_repair/geo_mark.h"

using namespace srcl_ctrl;
using namespace octomap;
using namespace cv;

int main(int argc, char* argv[])
{
	std::shared_ptr<octomap::OcTree> tree = std::make_shared<octomap::OcTree>(0.1);

	std::string tree_path = "/home/rdu/Workspace/srcl_rtk/srcl_ctrl/pc/planning/data/experiments/set1/local_octree.bt";
	tree->readBinary(tree_path);

	Mat input_image;
	std::string image_path = "/home/rdu/Workspace/srcl_rtk/srcl_ctrl/pc/planning/data/experiments/set1/map_path_repair.png";
	MapUtils::ReadImageFromFile(image_path, input_image);
	Map_t<SquareGrid> sgrid_map = SGridBuilder::BuildSquareGridMap(input_image, 32);
	sgrid_map.info.SetWorldSize(5.0, 5.0);
	sgrid_map.info.origin_offset_x = 2.5;
	sgrid_map.info.origin_offset_y = 2.5;
	std::shared_ptr<Graph_t<SquareCell*>> map_graph = GraphBuilder::BuildFromSquareGrid(sgrid_map.data_model,true);

	std::cout << "\n*********************************************************\n" << std::endl;

	std::cout << "num of leaf nodes: " << tree->getNumLeafNodes() << std::endl;
	std::cout << "tree depth: " <<  tree->getTreeDepth() << std::endl;

	std::cout << "\n---------------------------------------------------------\n" << std::endl;

	std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctree(tree);
	std::shared_ptr<Graph<const CubeCell&>> cubegraph = GraphBuilder::BuildFromCubeArray(cubearray);

	Graph_t<GeoMark> comb_graph;

	GeoMark mark1, mark2;
	uint64_t comb_graph_idx = 0;
	for(auto& edge:cubegraph->GetGraphEdges())
	{
		mark1.data_id_ = comb_graph_idx++;
		mark1.position = edge.src_->bundled_data_.location_;
		mark1.source = GeoMarkSource::LASER_OCTOMAP;
		mark1.source_id = edge.src_->bundled_data_.data_id_;

		mark2.data_id_ = comb_graph_idx++;
		mark2.position = edge.dst_->bundled_data_.location_;
		mark2.source = GeoMarkSource::LASER_OCTOMAP;
		mark2.source_id = edge.dst_->bundled_data_.data_id_;

		comb_graph.AddEdge(mark1, mark2, edge.cost_);
	}

	/*-------------------------------------------------------------------------------------*/
	// send data for visualization
	std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

	if(!lcm->good())
	{
		std::cout << "ERROR: Failed to initialize LCM." << std::endl;
		return -1;
	}

	srcl_msgs::Graph_t graph_msg;

	graph_msg.vertex_num = cubegraph->GetGraphVertices().size();
	for(auto& vtx : cubegraph->GetGraphVertices())
	{
		srcl_msgs::Vertex_t vertex;
		vertex.id = vtx->vertex_id_;

		vertex.position[0] = vtx->bundled_data_.location_.x;
		vertex.position[1] = vtx->bundled_data_.location_.y;
		vertex.position[2] = vtx->bundled_data_.location_.z;

		graph_msg.vertices.push_back(vertex);
	}

	graph_msg.edge_num = cubegraph->GetGraphUndirectedEdges().size();
	for(auto& eg : cubegraph->GetGraphUndirectedEdges())
	{
		srcl_msgs::Edge_t edge;
		edge.id_start = eg.src_->vertex_id_;
		edge.id_end = eg.dst_->vertex_id_;

		graph_msg.edges.push_back(edge);
	}

	lcm->publish("quad/cube_graph", &graph_msg);

	srcl_msgs::Graph_t graph_msg2;

	graph_msg2.vertex_num = map_graph->GetGraphVertices().size();
	for(auto& vtx : map_graph->GetGraphVertices())
	{
		srcl_msgs::Vertex_t vertex;
		vertex.id = vtx->vertex_id_;

		Position2Dd ref_world_pos = MapUtils::CoordinatesFromMapToRefWorld(vtx->bundled_data_->location_, sgrid_map.info);
		vertex.position[0] = ref_world_pos.x;
		vertex.position[1] = ref_world_pos.y;

		graph_msg2.vertices.push_back(vertex);
	}

	graph_msg2.edge_num = map_graph->GetGraphUndirectedEdges().size();
	for(auto& eg : map_graph->GetGraphUndirectedEdges())
	{
		srcl_msgs::Edge_t edge;
		edge.id_start = eg.src_->vertex_id_;
		edge.id_end = eg.dst_->vertex_id_;

		graph_msg2.edges.push_back(edge);
	}

	lcm->publish("quad/quad_planner_graph", &graph_msg2);

	srcl_msgs::Graph_t graph_msg3;

	graph_msg2.vertex_num = map_graph->GetGraphVertices().size();
	for(auto& vtx : map_graph->GetGraphVertices())
	{
		srcl_msgs::Vertex_t vertex;
		vertex.id = vtx->vertex_id_;

		Position2Dd ref_world_pos = MapUtils::CoordinatesFromMapToRefWorld(vtx->bundled_data_->location_, sgrid_map.info);
		vertex.position[0] = ref_world_pos.x;
		vertex.position[1] = ref_world_pos.y;

		graph_msg2.vertices.push_back(vertex);
	}

	graph_msg2.edge_num = map_graph->GetGraphUndirectedEdges().size();
	for(auto& eg : map_graph->GetGraphUndirectedEdges())
	{
		srcl_msgs::Edge_t edge;
		edge.id_start = eg.src_->vertex_id_;
		edge.id_end = eg.dst_->vertex_id_;

		graph_msg2.edges.push_back(edge);
	}
}


