/*
 * test_cubebuilder.cpp
 *
 *  Created on: Sep 8, 2016
 *      Author: rdu
 */

#include <memory>
#include <iostream>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/comm.hpp"

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include "geometry/cube_array/cube_array.h"
#include "geometry/cube_array_builder.h"
#include "geometry/graph_builder.h"

using namespace srcl_ctrl;
using namespace octomap;

void print_query_info(point3d query, OcTreeNode* node) {
	if (node != NULL) {
		std::cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << std::endl;
	}
	else
		std::cout << "occupancy probability at " << query << ":\t is unknown" << std::endl;
}

int main(int argc, char* argv[])
{
	std::shared_ptr<octomap::OcTree> tree = std::make_shared<octomap::OcTree>(0.1);

//	point3d endpoint ((float) 0.05f, (float) 0.05f, (float) 0.05f);
//	tree.updateNode(endpoint, true); // integrate 'occupied' measurement
//
//	point3d endpoint2 (0.05f, 0.05f, 0.15f);
//	tree.updateNode(endpoint2, false);  // integrate 'free' measurement

	//	std::string tree_path_ = "/home/rdu/Workspace/srcl_rtk/srcl_ctrl/pc/planning/data/octomap/local_3dmap.bt";
	//std::string tree_path_ = "/home/rdu/Workspace/srcl_rtk/srcl_ctrl/build/bin/test_tree.bt";
	std::string tree_path_ = "/home/rdu/Workspace/srcl_rtk/srcl_ctrl/build/bin/local_octree.bt";
	tree->readBinary(tree_path_);

	//tree.writeBinary("test_octomap_tree.bt");

	std::cout << "\n*********************************************************\n" << std::endl;
	std::cout << "num of leaf nodes: " << tree->getNumLeafNodes() << std::endl;
	std::cout << "tree depth: " <<  tree->getTreeDepth() << std::endl;

	std::cout << "\n---------------------------------------------------------\n" << std::endl;

	std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctree(tree);
	std::shared_ptr<Graph<CubeCell&>> cubegraph = GraphBuilder::BuildFromCubeArray(cubearray);

	std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

	if(!lcm->good())
	{
		std::cout << "ERROR: Failed to initialize LCM." << std::endl;
		return 1;
	}

	srcl_lcm_msgs::Graph_t graph_msg;

	graph_msg.vertex_num = cubegraph->GetGraphVertices().size();
	for(auto& vtx : cubegraph->GetGraphVertices())
	{
		srcl_lcm_msgs::Vertex_t vertex;
		vertex.id = vtx->vertex_id_;

		vertex.position[0] = vtx->bundled_data_.location_.x;
		vertex.position[1] = vtx->bundled_data_.location_.y;
		vertex.position[2] = vtx->bundled_data_.location_.z;

		graph_msg.vertices.push_back(vertex);
	}

	graph_msg.edge_num = cubegraph->GetGraphUndirectedEdges().size();
	for(auto& eg : cubegraph->GetGraphUndirectedEdges())
	{
		srcl_lcm_msgs::Edge_t edge;
		edge.id_start = eg.src_->vertex_id_;
		edge.id_end = eg.dst_->vertex_id_;

		graph_msg.edges.push_back(edge);
	}

	lcm->publish("quad/cube_graph", &graph_msg);
}


