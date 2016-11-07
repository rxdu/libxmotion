/*
 * octomap_server.cpp
 *
 *  Created on: May 26, 2016
 *      Author: rdu
 */

#include <iostream>
#include <sstream>

#include "geometry/cube_array/cube_array.h"
#include "geometry/cube_array_builder.h"
#include "geometry/graph_builder.h"
#include "local3d/octomap_server.h"

using namespace srcl_ctrl;

OctomapServer::OctomapServer(std::shared_ptr<lcm::LCM> lcm):
		lcm_(lcm),
		octree_res_(0.35),
		save_tree_(false),
		loop_count_(0),
		save_tree_name_("saved_octree.bt"),
		scan_buffer_size_(10)
{
//	sensor_origin_.x() = -1.8;
//	sensor_origin_.y() = 0.6;
//	sensor_origin_.z() = 0.8;//0.11;
	sensor_origin_.x() = 0;
	sensor_origin_.y() = 0;
	sensor_origin_.z() = 0.11;

	pc_snaps_.reserve(scan_buffer_size_);

	lcm_->subscribe("quad_data/laser_scan_points",&OctomapServer::LcmLaserScanPointsHandler, this);
	lcm_->subscribe("quad_data/quad_transform",&OctomapServer::LcmTransformHandler, this);
}

OctomapServer::~OctomapServer()
{
}

void OctomapServer::SaveTreeToFile(std::string file_name)
{
	save_tree_name_ = file_name;
	save_tree_ = true;
}

void OctomapServer::LcmTransformHandler(
		const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const srcl_lcm_msgs::QuadrotorTransform* msg)
{
	pos_ = Position3Dd(msg->base_to_world.position[0],msg->base_to_world.position[1],msg->base_to_world.position[2]);
	quat_ = Eigen::Quaterniond(msg->base_to_world.quaternion[0] , msg->base_to_world.quaternion[1] , msg->base_to_world.quaternion[2] , msg->base_to_world.quaternion[3]);

	transf_.trans = utils::Transformation::Translation3D(pos_.x,pos_.y,pos_.z);
	transf_.quat = quat_;

	base_pose_.trans() = octomath::Vector3(pos_.x, pos_.y, pos_.z);
	base_pose_.rot() = octomath::Quaternion(quat_.w(), quat_.x(), quat_.y(), quat_.z());
}

void OctomapServer::LcmLaserScanPointsHandler(
		const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const srcl_lcm_msgs::LaserScanPoints_t* msg)
{
	std::cout << msg->points.size() << " points received " << std::endl;
	loop_count_++;

	std::shared_ptr<octomap::OcTree> octree = std::make_shared<octomap::OcTree>(octree_res_);
	octree->setProbHit(0.7);
	octree->setProbMiss(0.4);
	octree->setClampingThresMin(0.12);
	octree->setClampingThresMax(0.97);

	base_pose_.trans() = octomath::Vector3(msg->pose.base_to_world.position[0],
			msg->pose.base_to_world.position[1],
			msg->pose.base_to_world.position[2]);// + msg->pose.laser_to_base.position[2]);
	//	base_pose_.trans() = octomath::Vector3(-1.8,0.6,0.91);
	base_pose_.rot() = octomath::Quaternion(msg->pose.base_to_world.quaternion[0],
			msg->pose.base_to_world.quaternion[1],
			msg->pose.base_to_world.quaternion[2],
			msg->pose.base_to_world.quaternion[3]);

	// octomap::Pointcloud pc;
	octomap::point3d start_point;

	start_point.x() = 0;
	start_point.y() = 0;
	start_point.z() = 0;
//	start_point.x() = base_pose_.trans().x();
//	start_point.y() = base_pose_.trans().y();
//	start_point.z() = base_pose_.trans().z();
	for(auto& pt : msg->points)
	{
		octomap::point3d end_point;
//		Position3Dd end_w = utils::Transformation::TransformPosition3D(transf_, Position3Dd(pt.x, pt.y, pt.z));
//		end_point.x() = end_w.x;
//		end_point.y() = end_w.y;
//		end_point.z() = end_w.z;
		end_point.x() = pt.x;
		end_point.y() = pt.y;
		end_point.z() = pt.z;

		octree->insertRay(start_point, end_point);

		//pc.push_back(end_point);
	}

//	PointCloudSnap pcs;
//	pcs.pc = pc;
//	pcs.trans = base_pose_;
//	if(pc_snaps_.size() < scan_buffer_size_)
//		pc_snaps_.push_back(pcs);
//	else
//	{
//		static int idx = 0;
//		pc_snaps_[(idx++)%10] = pcs;
//	}
//
//	for(auto& snap : pc_snaps_)
//	{
//		octomap::ScanNode  scan_node(&snap.pc, snap.trans, loop_count_);
//		octree->insertPointCloud(pc, sensor_origin_);
//	}

	//pc.transform(base_pose_);
//	current_pc_ = pc;
//	scan_node_ = octomap::ScanNode(&current_pc_, base_pose_, loop_count_);

//	if(point_cloud_.size() < scan_buffer_size_)
//		point_cloud_.push_back(pc);
//	else
//	{
//		point_cloud_.erase(point_cloud_.begin());
//		point_cloud_.push_back(pc);
//	}
//
//	// assemble the scans into one point cloud
//	octomap::Pointcloud accumulated_pc;
//	for(auto& p : point_cloud_)
//		accumulated_pc.push_back(p);

//	octree->insertPointCloud(pc, sensor_origin_);
//	octree->insertPointCloud(pc, sensor_origin_, base_pose_);

//	std::cout << "current pose: " << base_pose_.trans().x() << " , "
//			<< base_pose_.trans().y() << " , "
//			<< base_pose_.trans().z() << std::endl;

	octree->prune();

	octree_ = octree;

	srcl_lcm_msgs::Octomap_t octomap_msg;

	octomap_msg.binary = true;
	octomap_msg.resolution = octree_res_;
	octomap_msg.id = octree->getTreeType();

	std::stringstream datastream;

	if (octree->writeBinaryData(datastream))
	{
		std::string datastring = datastream.str();
		octomap_msg.data = std::vector<int8_t>(datastring.begin(), datastring.end());
		octomap_msg.data_size = octomap_msg.data.size();
	}

	srcl_lcm_msgs::NewDataReady_t notice_msg;
	notice_msg.new_data_ready_ = 1;
	lcm_->publish("quad_planner/new_octomap_ready", &notice_msg);

	lcm_->publish("quad_planner/hummingbird_laser_octomap", &octomap_msg);

//	// %10 : 10ms * 10 - 10 Hz update rate
//	if(loop_count_ % 10)
//	{
//		std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctree(octree);
//		std::shared_ptr<Graph<CubeCell&>> cubegraph = GraphBuilder::BuildFromCubeArray(cubearray);
//
////		std::cout << "tree size: " << octree_->size() << std::endl;
////		std::cout << "tree cube array size: " << cubearray->cubes_.size() << std::endl;
////		std::cout << "tree graph size: " << cubegraph->GetGraphVertices().size() << std::endl;
//
//		srcl_lcm_msgs::Graph_t graph_msg;
//
//		graph_msg.vertex_num = cubegraph->GetGraphVertices().size();
//		for(auto& vtx : cubegraph->GetGraphVertices())
//		{
//			srcl_lcm_msgs::Vertex_t vertex;
//			vertex.id = vtx->vertex_id_;
//
//			Position3Dd start_w = utils::Transformation::TransformPosition3D(transf_, Position3Dd(vtx->bundled_data_.location_.x,
//					vtx->bundled_data_.location_.y, vtx->bundled_data_.location_.z));
////			vertex.position[0] = vtx->bundled_data_.location_.x;
////			vertex.position[1] = vtx->bundled_data_.location_.y;
////			vertex.position[2] = vtx->bundled_data_.location_.z;
//			vertex.position[0] = start_w.x;
//			vertex.position[1] = start_w.y;
//			vertex.position[2] = start_w.z;
//
//			if(vtx->bundled_data_.occu_ != OccupancyType::OCCUPIED)
//				graph_msg.vertices.push_back(vertex);
//		}
//
//		graph_msg.edge_num = cubegraph->GetGraphUndirectedEdges().size();
//		for(auto& eg : cubegraph->GetGraphUndirectedEdges())
//		{
//			srcl_lcm_msgs::Edge_t edge;
//			edge.id_start = eg.src_->vertex_id_;
//			edge.id_end = eg.dst_->vertex_id_;
//
//			graph_msg.edges.push_back(edge);
//		}
//
//		lcm_->publish("quad_planner/cube_graph", &graph_msg);
//
//		loop_count_ = 0;
//	}

	if(save_tree_)
	{
		octree->writeBinary(save_tree_name_);
		save_tree_ = false;
	}
}
