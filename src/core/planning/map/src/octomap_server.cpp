/* 
 * octomap_server.cpp
 * 
 * Created on: May 26, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include <iostream>
#include <sstream>

#include "map/octomap_server.h"
#include "geometry/cube_array.hpp"
#include "map/cube_array_builder.h"
// #include "map/graph_builder.hpp"

using namespace librav;
using namespace octomap;

OctomapServer::OctomapServer(std::shared_ptr<lcm::LCM> lcm):
		lcm_(lcm),
		octree_res_(0.179),	// 0.3 -> 0.5
		save_tree_(false),
		loop_count_(0),
		save_tree_name_("saved_octree.bt"),
		scan_buffer_size_(10)
{
	pc_snaps_.reserve(scan_buffer_size_);
	pc_snaps_.resize(scan_buffer_size_);

	lcm_->subscribe("quad_data/laser_scan_points",&OctomapServer::LcmLaserScanPointsHandler, this);
}

OctomapServer::~OctomapServer()
{
}

void OctomapServer::SaveTreeToFile(std::string file_name)
{
	save_tree_name_ = file_name;
	save_tree_ = true;
}

bool OctomapServer::IsPositionOccupied(Position3Dd pos)
{
	point3d query (pos.x, pos.y, pos.z);

	OcTreeNode* result = octree_->search (query);

	if (result != NULL) {
		if(result->getOccupancy() <= octree_->getProbMiss() && result->getOccupancy() > octree_->getClampingThresMin())
			return false;
	}

	return true;
}

void OctomapServer::insertScan(const PointCloudSnap& pcs, std::shared_ptr<octomap::OcTree> m_octree)
{
	octomap::OcTreeKey m_updateBBXMin;
	octomap::OcTreeKey m_updateBBXMax;
	double m_maxRange = -1.0;	// no limit on ray range
	octomap::KeyRay m_keyRay;   // temp storage for ray casting

	point3d sensorOrigin = pcs.sensor_origin;

	if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)
			|| !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
	{
		//ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
		std::cout << "Could not generate Key for origin" << std::endl;
	}

	// instead of direct scan insertion, compute update to filter ground:
	KeySet free_cells, occupied_cells;

	// all other points: free on ray, occupied on endpoint:
	for (auto it = pcs.pc.begin(); it != pcs.pc.end(); ++it){
		point3d point(it->x(), it->y(), it->z());

		// maxrange check
		if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {

			// free cells
			if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
				free_cells.insert(m_keyRay.begin(), m_keyRay.end());
			}
			// occupied endpoint
			OcTreeKey key;
			if (m_octree->coordToKeyChecked(point, key)){
				occupied_cells.insert(key);

				updateMinKey(key, m_updateBBXMin);
				updateMaxKey(key, m_updateBBXMax);
			}
		} else {// ray longer than maxrange:;
			point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
			if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)){
				free_cells.insert(m_keyRay.begin(), m_keyRay.end());

				octomap::OcTreeKey endKey;
				if (m_octree->coordToKeyChecked(new_end, endKey)){
					free_cells.insert(endKey);
					updateMinKey(endKey, m_updateBBXMin);
					updateMaxKey(endKey, m_updateBBXMax);
				} else{
					//ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
					std::cout << "Could not generate Key for endpoint " << std::endl;
				}
			}
		}
	}

	// mark free cells only if not seen occupied in this cloud
	for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
		if (occupied_cells.find(*it) == occupied_cells.end()){
			m_octree->updateNode(*it, false);
		}
	}

	// now mark all occupied cells:
	for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
		m_octree->updateNode(*it, true);
	}

	// TODO: eval lazy+updateInner vs. proper insertion
	// non-lazy by default (updateInnerOccupancy() too slow for large maps)
	//m_octree->updateInnerOccupancy();
	octomap::point3d minPt, maxPt;
	//ROS_DEBUG_STREAM("Bounding box keys (before): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);

	// TODO: we could also limit the bbx to be within the map bounds here (see publishing check)
	minPt = m_octree->keyToCoord(m_updateBBXMin);
	maxPt = m_octree->keyToCoord(m_updateBBXMax);
	//ROS_DEBUG_STREAM("Updated area bounding box: "<< minPt << " - "<<maxPt);
	//ROS_DEBUG_STREAM("Bounding box keys (after): " << m_updateBBXMin[0] << " " <<m_updateBBXMin[1] << " " << m_updateBBXMin[2] << " / " <<m_updateBBXMax[0] << " "<<m_updateBBXMax[1] << " "<< m_updateBBXMax[2]);

	//if (m_compressMap)
	//m_octree->prune();
}

void OctomapServer::LcmLaserScanPointsHandler(
		const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const srcl_lcm_msgs::LaserScanPoints_t* msg)
{
	std::cout << msg->points.size() << " points received " << std::endl;

	// create a new octree
	std::shared_ptr<octomap::OcTree> octree = std::make_shared<octomap::OcTree>(octree_res_);
	octree->setProbHit(0.7);
	octree->setProbMiss(0.4);
	octree->setClampingThresMin(0.12);
	octree->setClampingThresMax(0.97);

	// get the transformation to transform points
	octree_transf_.trans = TransMath::Translation3D(msg->pose.base_to_world.position[0],
			msg->pose.base_to_world.position[1],
			msg->pose.base_to_world.position[2]+msg->pose.laser_to_base.position[2]);
	octree_transf_.quat = Eigen::Quaterniond(msg->pose.base_to_world.quaternion[0],
			msg->pose.base_to_world.quaternion[1],
			msg->pose.base_to_world.quaternion[2],
			msg->pose.base_to_world.quaternion[3]);

	// transform points and save them to a point cloud
	octomap::Pointcloud pc;
	for(auto& pt : msg->points)
	{
		octomap::point3d end_point;

		Position3Dd end_w = TransMath::TransformPosition3D(octree_transf_, Position3Dd(pt.x, pt.y, pt.z));
		end_point.x() = end_w.x;
		end_point.y() = end_w.y;
		end_point.z() = end_w.z;

		pc.push_back(end_point);
	}

	// save the data as a point cloud snap
	PointCloudSnap pcs;
	pcs.pc = pc;
	pcs.transf.trans() = octomath::Vector3(msg->pose.base_to_world.position[0],
			msg->pose.base_to_world.position[1],
			msg->pose.base_to_world.position[2]+msg->pose.laser_to_base.position[2]);
	pcs.transf.rot() = octomath::Quaternion(msg->pose.base_to_world.quaternion[0],
			msg->pose.base_to_world.quaternion[1],
			msg->pose.base_to_world.quaternion[2],
			msg->pose.base_to_world.quaternion[3]);
	pcs.sensor_origin = pcs.transf.trans();

	static int idx = 0;
	pc_snaps_[(idx++)%10] = pcs;

	// insert all saved data into the octree
	for(auto& snap : pc_snaps_)
	{
		if(snap.pc.size() != 0)
			octree->insertPointCloud(pc, pcs.sensor_origin);
//			insertScan(snap, octree);
	}

	octree->prune();

	octree_ = octree;

	// %10 : 10ms * 10 - 10 Hz update rate
	if(loop_count_++ % 10)
	{
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

//		std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctree(octree);
//		std::shared_ptr<Graph<CubeCell&>> cubegraph = GraphBuilder::BuildFromCubeArray(cubearray);

//		std::cout << "tree size: " << octree_->size() << std::endl;
//		std::cout << "tree cube array size: " << cubearray->cubes_.size() << std::endl;
//		std::cout << "tree graph size: " << cubegraph->GetGraphVertices().size() << std::endl;

//		std::cout << "quad position: " << msg->pose.base_to_world.position[0] << " , "
//				<< msg->pose.base_to_world.position[1] << " , "
//				<< msg->pose.base_to_world.position[2] << std::endl;
//		uint64_t cid = cubearray->GetIDFromPosition(msg->pose.base_to_world.position[0],
//				msg->pose.base_to_world.position[1], msg->pose.base_to_world.position[2]);
//
//		if(cubearray->isIDValid(cid))
//		{
//			std::cout << "cube id: " << cid << std::endl;
//			std::cout << "cube info: " << cubearray->cubes_[cid].location_.x << " , "
//						<< cubearray->cubes_[cid].location_.y << " , "
//						<< cubearray->cubes_[cid].location_.z << std::endl;
//		}
//		else
//			std::cout << "****************************** error ******************************" << std::endl;

//		srcl_lcm_msgs::Graph_t graph_msg;
//
//		graph_msg.vertex_num = cubegraph->GetGraphVertices().size();
//		for(auto& vtx : cubegraph->GetGraphVertices())
//		{
//			srcl_lcm_msgs::Vertex_t vertex;
//			vertex.id = vtx->vertex_id_;
//
////			Position3Dd start_w = utils::Transformation::TransformPosition3D(transf_, Position3Dd(vtx->bundled_data_.location_.x,
////					vtx->bundled_data_.location_.y, vtx->bundled_data_.location_.z));
////			vertex.position[0] = start_w.x;
////			vertex.position[1] = start_w.y;
////			vertex.position[2] = start_w.z;
//			vertex.position[0] = vtx->bundled_data_.location_.x;
//			vertex.position[1] = vtx->bundled_data_.location_.y;
//			vertex.position[2] = vtx->bundled_data_.location_.z;
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
	}

	if(save_tree_)
	{
		octree->writeBinary(save_tree_name_);
		std::cout << "************************ octomap saved ************************" << std::endl;
		save_tree_ = false;
	}
}

//void OctomapServer::LcmLaserScanPointsHandler(
//		const lcm::ReceiveBuffer* rbuf,
//		const std::string& chan,
//		const srcl_lcm_msgs::LaserScanPoints_t* msg)
//{
//	std::cout << msg->points.size() << " points received " << std::endl;
//
//	std::shared_ptr<octomap::OcTree> octree = std::make_shared<octomap::OcTree>(octree_res_);
//	octree->setProbHit(0.7);
//	octree->setProbMiss(0.4);
//	octree->setClampingThresMin(0.12);
//	octree->setClampingThresMax(0.97);
//
////	base_pose_.trans() = octomath::Vector3(msg->pose.base_to_world.position[0],
////			msg->pose.base_to_world.position[1],
////			msg->pose.base_to_world.position[2]);// + msg->pose.laser_to_base.position[2]);
////	//	base_pose_.trans() = octomath::Vector3(-1.8,0.6,0.91);
////	base_pose_.rot() = octomath::Quaternion(msg->pose.base_to_world.quaternion[0],
////			msg->pose.base_to_world.quaternion[1],
////			msg->pose.base_to_world.quaternion[2],
////			msg->pose.base_to_world.quaternion[3]);
//
//	octree_transf_.trans = utils::Transformation::Translation3D(msg->pose.base_to_world.position[0],
//			msg->pose.base_to_world.position[1], msg->pose.base_to_world.position[2]);
//	octree_transf_.quat = Eigen::Quaterniond(msg->pose.base_to_world.quaternion[0],
//			msg->pose.base_to_world.quaternion[1], msg->pose.base_to_world.quaternion[2],
//			msg->pose.base_to_world.quaternion[3]);
//
//	octomap::Pointcloud pc;
//	octomap::point3d start_point;
////	start_point.x() = 0;
////	start_point.y() = 0;
////	start_point.z() = 0;
////	start_point.x() = base_pose_.trans().x();
////	start_point.y() = base_pose_.trans().y();
////	start_point.z() = base_pose_.trans().z();
////	Position3Dd start_w = utils::Transformation::TransformPosition3D(octree_transf_, Position3Dd(msg->pose.base_to_world.position[0],
////			msg->pose.base_to_world.position[1], msg->pose.base_to_world.position[2]));
//	start_point.x() = msg->pose.base_to_world.position[0];
//	start_point.y() = msg->pose.base_to_world.position[1];
//	start_point.z() = msg->pose.base_to_world.position[2];
//	for(auto& pt : msg->points)
//	{
//		octomap::point3d end_point;
//
//		Position3Dd end_w = utils::Transformation::TransformPosition3D(octree_transf_, Position3Dd(pt.x, pt.y, pt.z));
//		end_point.x() = end_w.x;
//		end_point.y() = end_w.y;
//		end_point.z() = end_w.z;
////		end_point.x() = pt.x;
////		end_point.y() = pt.y;
////		end_point.z() = pt.z;
//
//		octree->insertRay(start_point, end_point);
//
////		pc.push_back(end_point);
//	}
//
////	PointCloudSnap pcs;
////	pcs.pc = pc;
////	pcs.trans = base_pose_;
////	if(pc_snaps_.size() < scan_buffer_size_)
////		pc_snaps_.push_back(pcs);
////	else
////	{
////		static int idx = 0;
////		pc_snaps_[(idx++)%10] = pcs;
////	}
////
////	for(auto& snap : pc_snaps_)
////	{
////		octomap::ScanNode  scan_node(&snap.pc, snap.trans, loop_count_);
////		octree->insertPointCloud(pc, sensor_origin_);
////	}
//
//	//pc.transform(base_pose_);
////	current_pc_ = pc;
////	scan_node_ = octomap::ScanNode(&current_pc_, base_pose_, loop_count_);
//
////	if(point_cloud_.size() < scan_buffer_size_)
////		point_cloud_.push_back(pc);
////	else
////	{
////		point_cloud_.erase(point_cloud_.begin());
////		point_cloud_.push_back(pc);
////	}
////
////	// assemble the scans into one point cloud
////	octomap::Pointcloud accumulated_pc;
////	for(auto& p : point_cloud_)
////		accumulated_pc.push_back(p);
//
//	base_pose_.trans() = octomath::Vector3(msg->pose.base_to_world.position[0],
//			msg->pose.base_to_world.position[1], msg->pose.base_to_world.position[2]);
//	base_pose_.rot() = octomath::Quaternion(msg->pose.base_to_world.quaternion[0],
//			msg->pose.base_to_world.quaternion[1], msg->pose.base_to_world.quaternion[2],
//			msg->pose.base_to_world.quaternion[3]);
//	sensor_origin_.x() = 0;//msg->pose.base_to_world.position[0];
//	sensor_origin_.y() = 0;//msg->pose.base_to_world.position[1];
//	sensor_origin_.z() = 0.11;//msg->pose.base_to_world.position[2]; //0.11;
//	//octree->insertPointCloud(pc, sensor_origin_);
//	//octree->insertPointCloud(pc, sensor_origin_, base_pose_);
//
//	octree->prune();
//
//	octree_ = octree;
//
//	// %10 : 10ms * 10 - 10 Hz update rate
//	if(loop_count_++ % 10)
//	{
//		srcl_lcm_msgs::Octomap_t octomap_msg;
//
//		octomap_msg.binary = true;
//		octomap_msg.resolution = octree_res_;
//		octomap_msg.id = octree->getTreeType();
//
//		std::stringstream datastream;
//
//		if (octree->writeBinaryData(datastream))
//		{
//			std::string datastring = datastream.str();
//			octomap_msg.data = std::vector<int8_t>(datastring.begin(), datastring.end());
//			octomap_msg.data_size = octomap_msg.data.size();
//		}
//
//		srcl_lcm_msgs::NewDataReady_t notice_msg;
//		notice_msg.new_data_ready_ = 1;
//		lcm_->publish("quad_planner/new_octomap_ready", &notice_msg);
//
//		lcm_->publish("quad_planner/hummingbird_laser_octomap", &octomap_msg);
//
////		std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctree(octree);
////		std::shared_ptr<Graph<CubeCell&>> cubegraph = GraphBuilder::BuildFromCubeArray(cubearray);
////
//////		std::cout << "tree size: " << octree_->size() << std::endl;
//////		std::cout << "tree cube array size: " << cubearray->cubes_.size() << std::endl;
//////		std::cout << "tree graph size: " << cubegraph->GetGraphVertices().size() << std::endl;
////
////		srcl_lcm_msgs::Graph_t graph_msg;
////
////		graph_msg.vertex_num = cubegraph->GetGraphVertices().size();
////		for(auto& vtx : cubegraph->GetGraphVertices())
////		{
////			srcl_lcm_msgs::Vertex_t vertex;
////			vertex.id = vtx->vertex_id_;
////
////			Position3Dd start_w = utils::Transformation::TransformPosition3D(transf_, Position3Dd(vtx->bundled_data_.location_.x,
////					vtx->bundled_data_.location_.y, vtx->bundled_data_.location_.z));
//////			vertex.position[0] = vtx->bundled_data_.location_.x;
//////			vertex.position[1] = vtx->bundled_data_.location_.y;
//////			vertex.position[2] = vtx->bundled_data_.location_.z;
////			vertex.position[0] = start_w.x;
////			vertex.position[1] = start_w.y;
////			vertex.position[2] = start_w.z;
////
////			if(vtx->bundled_data_.occu_ != OccupancyType::OCCUPIED)
////				graph_msg.vertices.push_back(vertex);
////		}
////
////		graph_msg.edge_num = cubegraph->GetGraphUndirectedEdges().size();
////		for(auto& eg : cubegraph->GetGraphUndirectedEdges())
////		{
////			srcl_lcm_msgs::Edge_t edge;
////			edge.id_start = eg.src_->vertex_id_;
////			edge.id_end = eg.dst_->vertex_id_;
////
////			graph_msg.edges.push_back(edge);
////		}
////
////		lcm_->publish("quad_planner/cube_graph", &graph_msg);
//	}
//
//	if(save_tree_)
//	{
//		octree->writeBinary(save_tree_name_);
//		save_tree_ = false;
//	}
//}
