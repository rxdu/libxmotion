/*
 * octomap_server.cpp
 *
 *  Created on: May 26, 2016
 *      Author: rdu
 */

#include <iostream>
#include <sstream>

#include "geometry/cube_array/cube_array.h"
#include "local3d/octomap_server.h"
#include "geometry/cube_array_builder.h"
#include "geometry/graph_builder.h"

using namespace srcl_ctrl;

OctomapServer::OctomapServer(std::shared_ptr<lcm::LCM> lcm):
		lcm_(lcm),
		octree_res_(0.35),
		save_tree_(false),
		loop_count_(0)
{
	lcm_->subscribe("vis_data_laser_scan_points",&OctomapServer::LcmLaserScanPointsHandler, this);
}

OctomapServer::~OctomapServer()
{
}

void OctomapServer::SaveTreeToFile()
{
	//octree_->writeBinary("test_tree.bt");
	save_tree_ = true;
}

void OctomapServer::LcmLaserScanPointsHandler(
		const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const srcl_msgs::LaserScanPoints_t* msg)
{
	std::cout << "points received " << std::endl;
	loop_count_++;

	std::shared_ptr<octomap::OcTree> octree = std::make_shared<octomap::OcTree>(octree_res_);
	octree->setProbHit(0.7);
	octree->setProbMiss(0.4);
	octree->setClampingThresMin(0.12);
	octree->setClampingThresMax(0.97);

	for(auto& pt : msg->points)
	{
		octomap::point3d origin, end;

		origin.x() = 0;
		origin.y() = 0;
		origin.z() = 0;

		end.x() = pt.x;
		end.y() = pt.y;
		end.z() = pt.z;

		octree->insertRay(origin, end);
	}

	srcl_msgs::Octomap_t octomap_msg;

	octomap_msg.binary = true;
	octomap_msg.resolution = octree_res_;
	octomap_msg.id = octree->getTreeType();

	octree->prune();
	std::stringstream datastream;

	if (octree->writeBinaryData(datastream))
	{
		std::string datastring = datastream.str();
		octomap_msg.data = std::vector<int8_t>(datastring.begin(), datastring.end());
		octomap_msg.data_size = octomap_msg.data.size();
	}

	lcm_->publish("hummingbird_laser_octomap", &octomap_msg);

	if(loop_count_ > 15)
	{
		std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctree(octree);
		std::shared_ptr<Graph<const CubeCell&>> cubegraph = GraphBuilder::BuildFromCubeArray(cubearray);

		srcl_msgs::Graph_t graph_msg;

		graph_msg.vertex_num = cubegraph->GetGraphVertices().size();
		for(auto& vtx : cubegraph->GetGraphVertices())
		{
			srcl_msgs::Vertex_t vertex;
			vertex.id = vtx->vertex_id_;

			vertex.position[0] = vtx->bundled_data_.location_.x;
			vertex.position[1] = vtx->bundled_data_.location_.y;
			vertex.position[2] = vtx->bundled_data_.location_.z;

			if(vtx->bundled_data_.occu_ != OccupancyType::OCCUPIED)
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

		lcm_->publish("quad/cube_graph", &graph_msg);
		loop_count_ = 0;
	}

	if(save_tree_)
	{
		octree->writeBinary("local_octree.bt");
		save_tree_ = false;
	}
}
