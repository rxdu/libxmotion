/*
 * quad_path_repair.cpp
 *
 *  Created on: Sep 9, 2016
 *      Author: rdu
 */

#include <iostream>
#include <ctime>
#include <cmath>
#include <limits>

#include "eigen3/Eigen/Geometry"

#ifdef ENABLE_G3LOG
#include "ctrl_utils/logging/logging_helper.h"
#endif

#include "path_repair/quad_path_repair.h"
#include "map/map_utils.h"
#include "geometry/cube_array/cube_array.h"
#include "geometry/cube_array_builder.h"
#include "geometry/graph_builder.h"

using namespace srcl_ctrl;

QuadPathRepair::QuadPathRepair(std::shared_ptr<lcm::LCM> lcm):
		lcm_(lcm),
		octomap_server_(OctomapServer(lcm_)),
		mission_tracker_(new MissionTracker(lcm_)),
		active_graph_planner_(GraphPlannerType::NOT_SPECIFIED),
		sensor_range_(5.0),
		current_sys_time_(0),
		gstart_set_(false),
		ggoal_set_(false),
		world_size_set_(false),
		auto_update_pos_(true),
		est_new_dist_(std::numeric_limits<double>::infinity()),
		update_global_plan_(false)
{
	if(!lcm_->good())
		std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
	else {
		traj_gen_ = std::make_shared<TrajectoryGenerator>(lcm_);

		lcm_->subscribe("quad_data/quad_transform",&QuadPathRepair::LcmTransformHandler, this);
		lcm_->subscribe("quad_planner/new_octomap_ready",&QuadPathRepair::LcmOctomapHandler, this);
		lcm_->subscribe("quad_data/system_time", &QuadPathRepair::LcmSysTimeHandler, this);
	}
}

QuadPathRepair::~QuadPathRepair()
{

}

void QuadPathRepair::ConfigGraphPlanner(MapConfig config, double world_size_x, double world_size_y)
{
	if(config.GetMapType().data_model == MapDataModel::SQUARE_GRID)
	{
		bool result = sgrid_planner_.UpdateMapConfig(config);

		if(result)
		{
			std::cout << "square grid planner activated" << std::endl;
			active_graph_planner_ = GraphPlannerType::SQUAREGRID_PLANNER;

			// configure navigation field for shortcut analysis
			nav_field_ = std::make_shared<NavField<SquareCell*>>(sgrid_planner_.graph_);
			sc_evaluator_ = std::make_shared<ShortcutEval>(sgrid_planner_.map_.data_model, nav_field_);
		}
	}
	else
		return;

	// the world size must be set after the planner is updated, otherwise the configuration will be override
	sgrid_planner_.map_.info.SetWorldSize(world_size_x, world_size_y);
	world_size_set_ = true;

	sgrid_planner_.map_.info.resolution = sgrid_planner_.map_.info.world_size_x/sgrid_planner_.map_.info.map_size_x*sgrid_planner_.map_.data_model->cell_size_;
	std::cout << "sgrid map reso: " << sgrid_planner_.map_.info.resolution << std::endl;
	geomark_graph_.UpdateSquareGridInfo(sgrid_planner_.graph_, sgrid_planner_.map_);
	octomap_server_.SetOctreeResolution(sgrid_planner_.map_.info.resolution);

//	srcl_lcm_msgs::Graph_t graph_msg = GenerateLcmGraphMsg();
//	lcm_->publish("quad_planner/quad_planner_graph", &graph_msg);
}

void QuadPathRepair::SetStartMapPosition(Position2D pos)
{
	if(pos == start_pos_)
		return;

	start_pos_.x = pos.x;
	start_pos_.y = pos.y;

	gstart_set_ = true;

	//est_dist2goal_ = std::numeric_limits<double>::infinity();

	if(gstart_set_ && ggoal_set_)
		update_global_plan_ = true;
}

void QuadPathRepair::SetGoalMapPosition(Position2D pos)
{
	goal_pos_.x = pos.x;
	goal_pos_.y = pos.y;

	ggoal_set_ = true;

	auto goal_id = sgrid_planner_.map_.data_model->GetIDFromPosition(goal_pos_.x, goal_pos_.y);
	nav_field_->UpdateNavField(goal_id);
	// TODO update sensor range from calculation
	sc_evaluator_->EvaluateGridShortcutPotential(15);

	if(gstart_set_ && ggoal_set_)
		update_global_plan_ = true;
}

void QuadPathRepair::SetStartRefWorldPosition(Position2Dd pos)
{
//	std::cout << "\nposition in ref world: " << pos.x << " , " << pos.y << std::endl;

	Position2Dd mpos;
	mpos = MapUtils::CoordinatesFromRefWorldToMapWorld(pos, GetActiveMapInfo());
//	std::cout << "position in map world: " << mpos.x << " , " << mpos.y << std::endl;

	Position2D map_pos;
	map_pos = MapUtils::CoordinatesFromMapWorldToMap(mpos, GetActiveMapInfo());
//	std::cout << "position in map: " << map_pos.x << " , " << map_pos.y << std::endl;

	Position2D map_padded_pos;
	map_padded_pos = MapUtils::CoordinatesFromOriginalToPadded(map_pos, GetActiveMapInfo());
//	std::cout << "position in padded map: " << map_padded_pos.x << " , " << map_padded_pos.y << std::endl;

	SetStartMapPosition(map_padded_pos);
}

void QuadPathRepair::SetGoalRefWorldPosition(Position2Dd pos)
{
	Position2Dd mpos;
	mpos = MapUtils::CoordinatesFromRefWorldToMapWorld(pos, GetActiveMapInfo());

	Position2D map_pos;
	map_pos = MapUtils::CoordinatesFromMapWorldToMap(mpos, GetActiveMapInfo());

	Position2D map_padded_pos;
	map_padded_pos = MapUtils::CoordinatesFromOriginalToPadded(map_pos, GetActiveMapInfo());

	SetGoalMapPosition(map_padded_pos);
}

std::vector<Position2D> QuadPathRepair::UpdateGlobalPath()
{
	std::vector<Position2D> waypoints;

//	std::cout << "----> start: " << start_pos_.x << " , " << start_pos_.y << std::endl;
//	std::cout << "----> goal: " << goal_pos_.x << " , " << goal_pos_.y << std::endl;

	auto traj_vtx = sgrid_planner_.Search(start_pos_, goal_pos_);
	for(auto& wp:traj_vtx)
		waypoints.push_back(wp->bundled_data_->location_);

	srcl_lcm_msgs::Path_t path_msg = GenerateLcmPathMsg(waypoints);
	lcm_->publish("quad_planner/quad_planner_graph_path", &path_msg);

	update_global_plan_ = false;

	return waypoints;
}

std::vector<uint64_t> QuadPathRepair::UpdateGlobalPathID()
{
	std::vector<uint64_t> waypoints;

//		std::cout << "----> start: " << start_pos_.x << " , " << start_pos_.y << std::endl;
//		std::cout << "----> goal: " << goal_pos_.x << " , " << goal_pos_.y << std::endl;

	auto traj_vtx = sgrid_planner_.Search(start_pos_, goal_pos_);
	for(auto& wp:traj_vtx)
		waypoints.push_back(wp->vertex_id_);

	update_global_plan_ = false;

	return waypoints;
}

cv::Mat QuadPathRepair::GetActiveMap()
{
	return sgrid_planner_.map_.padded_image;
}

MapInfo QuadPathRepair::GetActiveMapInfo()
{
	MapInfo empty_info;

	return sgrid_planner_.map_.info;
}

void QuadPathRepair::LcmTransformHandler(
		const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const srcl_lcm_msgs::QuadrotorTransform* msg)
{
	Position2Dd rpos;
	rpos.x = msg->base_to_world.position[0];
	rpos.y = msg->base_to_world.position[1];

	if(auto_update_pos_)
		SetStartRefWorldPosition(rpos);

	geomark_graph_.UpdateVehiclePose(Position3Dd(msg->base_to_world.position[0],msg->base_to_world.position[1],msg->base_to_world.position[2]),
					Eigen::Quaterniond(msg->base_to_world.quaternion[0] , msg->base_to_world.quaternion[1] , msg->base_to_world.quaternion[2] , msg->base_to_world.quaternion[3]));
	mission_tracker_->UpdateCurrentPosition(Position3Dd(msg->base_to_world.position[0],msg->base_to_world.position[1],msg->base_to_world.position[2]));
}

void QuadPathRepair::LcmSysTimeHandler(
		const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const srcl_lcm_msgs::TimeStamp_t* msg)
{
	current_sys_time_ = msg->time_stamp;
}

bool QuadPathRepair::CheckPathSafety(std::shared_ptr<CubeArray> cube_array)
{
	// only check waypoints that is from 3D graph
	for(auto& wp : mission_tracker_->active_path_)
	{
		Position3Dd pt_pos_w = wp.position;

		if(octomap_server_.IsPositionOccupied(pt_pos_w))
			return false;
	}

	return true;
}

bool QuadPathRepair::EvaluateNewPath(std::vector<Position3Dd>& new_path)
{
	if(std::sqrt(std::pow(new_path.front().x - mission_tracker_->current_position_.x, 2) +
			std::pow(new_path.front().y - mission_tracker_->current_position_.y, 2) +
			std::pow(new_path.front().z - mission_tracker_->current_position_.z, 2)) > 0.35)
	{
		std::cout << "rejected plan due to wrong starting point" << std::endl;
		return false;
	}

	est_new_dist_ = 0;
	for(int i = 0; i < new_path.size() - 1; i++)
		est_new_dist_ += std::sqrt(std::pow(new_path[i].x - new_path[i + 1].x,2) +
				std::pow(new_path[i].y - new_path[i + 1].y,2));// +
				//std::pow(new_path[i].z - new_path[i + 1].z,2));

	std::cout << "new path dist: " << est_new_dist_ << " , remaining dist of current path: "
			<< mission_tracker_->remaining_path_length_ << std::endl;

	LOG(INFO) << "old_dist = " <<  mission_tracker_->remaining_path_length_
					<< " , new_dist = " << est_new_dist_;

	if(new_path.size() > 0 && est_new_dist_ < mission_tracker_->remaining_path_length_ * 0.85)
	{
		LOG(INFO) << " --------> new plan found <-------- ";
		LOG(INFO) << "remaining path length: " <<  mission_tracker_->remaining_path_length_
							<< " , new path length: " << est_new_dist_;
		return true;
	}
	else
		return false;
}

int32_t QuadPathRepair::FindFurthestPointWithinRadius(std::vector<Position3Dd>& new_path, double radius) const
{
	Position3Dd start = new_path.front();
	int32_t goal_idx = new_path.size() - 1;

	int32_t idx = 0;
	for(auto it = new_path.begin(); it != new_path.end() - 1; it++)
	{
		double dist1 = std::sqrt(std::pow((*it).x - start.x,2) +
				std::pow((*it).y - start.y,2) +
				std::pow((*it).z - start.z,2));
		double dist2 = std::sqrt(std::pow((*(it+1)).x - start.x,2) +
				std::pow((*(it+1)).y - start.y,2) +
				std::pow((*(it+1)).z - start.z,2));

		if(dist1 <= radius && dist2 > radius) {
			goal_idx = idx;
		}

		idx++;
	}

	return goal_idx;
}

//void QuadPathRepair::LcmOctomapHandler(
//		const lcm::ReceiveBuffer* rbuf,
//		const std::string& chan,
//		const srcl_lcm_msgs::NewDataReady_t* msg)
//{
//	std::cout << "\n---------------------- New Iteration -------------------------" << std::endl;
//
//	LOG(INFO) << "----------- New iteration -----------";
//
//	if(mission_tracker_->remaining_path_length_ < 0.5)
//	{
//		std::cout << "Getting close to goal, no need to replan" << std::endl;
//		LOG(INFO) << "Getting close to goal, no need to replan";
//		return;
//	}
//
//	static int count = 0;
//	srcl_lcm_msgs::KeyframeSet_t kf_cmd;
//
//	// record the planning time
//	kf_cmd.sys_time.time_stamp = current_sys_time_;
//
//	//LOG(INFO) << "Before build cube array and cube graph";
//
//	//std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctree(octomap_server_.octree_);
//	std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctreeWithExtObstacle(octomap_server_.octree_);
//	std::shared_ptr<Graph<CubeCell&>> cubegraph = GraphBuilder::BuildFromCubeArray(cubearray);
//
//	//LOG(INFO) << "After build cube array and cube graph";
//
//	// don't replan if 3d information is too limited
//	if(mission_tracker_->mission_started_ && (cubearray->cubes_.size() == 0 || cubegraph->GetGraphVertices().size() < 5))
//	{
//		std::cerr << "Too limited 3D information collected" << std::endl;
//		LOG(INFO) << "Too limited 3D information collected";
//		return;
//	}
//
//	//LOG(INFO) << "Before combining graphs";
//
//	//int64_t geo_start_id_astar = geomark_graph_.CombineBaseWithCubeArrayGraph(cubearray, cubegraph);//, octomap_server_.octree_transf_);
//	int64_t geo_start_id_astar = geomark_graph_.MergeCubeArrayInfo(cubegraph, cubearray);
//	//LOG(INFO) << "After combining graphs";
//
//	// don't replan if failed to combine graphs
//	if(geo_start_id_astar == -1)
//	{
//		std::cerr << "Failed to combine graphs" << std::endl;
//		LOG(INFO) << "Failed to combine graphs";
//		return;
//	}
//
//	LOG(INFO) << "Combined graph size: " << sgrid_planner_.graph_->GetGraphVertices().size();
//	std::cout << "Graph size (combined, 3d): " << sgrid_planner_.graph_->GetGraphVertices().size() << " , " << cubegraph->GetGraphVertices().size() << std::endl;
//
//	uint64_t map_goal_id = sgrid_planner_.map_.data_model->GetIDFromPosition(goal_pos_.x, goal_pos_.y);
//	uint64_t geo_goal_id_astar = sgrid_planner_.graph_->GetVertexFromID(map_goal_id)->bundled_data_->geo_mark_id_;
//
//	//LOG(INFO) << "Before a* search";
//
//	clock_t exec_time;
//	exec_time = clock();
//	auto comb_path = AStar::Search(geomark_graph_.combined_graph_, geo_start_id_astar, geo_goal_id_astar);
//	exec_time = clock() - exec_time;
//	std::cout << "Search in 3D finished in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;
//
//	//LOG(INFO) << "After a* search";
//
//	std::vector<Position3Dd> raw_wps;
//	for(auto& wp:comb_path)
//		raw_wps.push_back(wp->bundled_data_.position);
//
//	// if failed to find a 3d path, terminate this iteration
//	if(raw_wps.size() <= 1)
//		return;
//
//	std::vector<Position3Dd> selected_wps = MissionUtils::GetKeyTurningWaypoints(raw_wps);
//
//	 if(!mission_tracker_->mission_started_ || EvaluateNewPath(selected_wps))
//	 {
//	 	if(mission_tracker_->mission_started_)
//	 		std::cout << "-------- found better solution ---------" << std::endl;
//	 	else
//	 		mission_tracker_->mission_started_ = true;
//
//	 	// update mission tracking information
//	 	mission_tracker_->UpdateActivePathWaypoints(comb_path);
//	 	mission_tracker_->remaining_path_length_ = est_new_dist_;
//
//	 	kf_cmd.path_id = mission_tracker_->path_id_;
//
//	 	//**** Strategy Change ****//
//	 	//Eigen::Vector3d goal_vec(selected_wps.back().x, selected_wps.back().y, 0);
//	 	int32_t fpt_idx = FindFurthestPointWithinRadius(selected_wps, 5.0);
//	 	Eigen::Vector3d furthest_pt_vec(selected_wps[fpt_idx].x, selected_wps[fpt_idx].y, 0);
//	 	Eigen::Vector3d goal_vec(selected_wps.back().x, selected_wps.back().y, 0);
//
//	 	kf_cmd.kf_num = selected_wps.size();
//	 	int32_t wp_cnt = 0;
//	 	for(auto& wp:selected_wps)
//	 	{
//	 		srcl_lcm_msgs::Keyframe_t kf;
//	 		kf.vel_constr = false;
//
//	 		kf.positions[0] = wp.x;
//	 		kf.positions[1] = wp.y;
//	 		kf.positions[2] = wp.z;
//
//	 		Eigen::Vector3d pos_vec(wp.x, wp.y, 0);
//	 		Eigen::Vector3d dir_vec;
//	 		if(wp_cnt < fpt_idx)
//	 			dir_vec = furthest_pt_vec - pos_vec;
//	 		else
//	 			dir_vec = goal_vec - pos_vec;
//	 		Eigen::Vector3d x_vec(1,0,0);
//	 		double angle = - std::acos(dir_vec.normalized().dot(x_vec));
//	 		//std::cout<<"angle: " << angle << " , vec: " << dir_vec << std::endl;
//	 		kf.yaw = angle;
//
//	 		kf_cmd.kfs.push_back(kf);
//	 		wp_cnt++;
//	 	}
//	 	kf_cmd.kfs.front().yaw = 0;
//	 	kf_cmd.kfs.back().yaw = -M_PI/4;
//
//	 	lcm_->publish("quad_planner/goal_keyframe_set", &kf_cmd);
//
//	 	// send data for visualization
//	 	Send3DSearchPathToVis(selected_wps);
//	 }
//
////	if(count++ % 20 == 0)
////	{
////		Send3DSearchPathToVis(selected_wps);
////
////		srcl_lcm_msgs::Graph_t graph_msg;
////
////		// combined graph
////		graph_msg.vertex_num = gcombiner_.combined_graph_.GetGraphVertices().size();
////		for(auto& vtx : gcombiner_.combined_graph_.GetGraphVertices())
////		{
////			srcl_lcm_msgs::Vertex_t vertex;
////			vertex.id = vtx->vertex_id_;
////
////			vertex.position[0] = vtx->bundled_data_.position.x;
////			vertex.position[1] = vtx->bundled_data_.position.y;
////			vertex.position[2] = vtx->bundled_data_.position.z;
////
////			graph_msg.vertices.push_back(vertex);
////		}
////
////		graph_msg.edge_num = gcombiner_.combined_graph_.GetGraphUndirectedEdges().size();
////		for(auto& eg : gcombiner_.combined_graph_.GetGraphUndirectedEdges())
////		{
////			srcl_lcm_msgs::Edge_t edge;
////			edge.id_start = eg.src_->vertex_id_;
////			edge.id_end = eg.dst_->vertex_id_;
////
////			graph_msg.edges.push_back(edge);
////		}
////
////		// cube graph
//////		graph_msg.vertex_num = cubegraph->GetGraphVertices().size();
//////		for(auto& vtx : cubegraph->GetGraphVertices())
//////		{
//////			srcl_lcm_msgs::Vertex_t vertex;
//////			vertex.id = vtx->vertex_id_;
//////
//////			vertex.position[0] = vtx->bundled_data_.location_.x;
//////			vertex.position[1] = vtx->bundled_data_.location_.y;
//////			vertex.position[2] = vtx->bundled_data_.location_.z;
//////
//////			graph_msg.vertices.push_back(vertex);
//////		}
//////
//////		graph_msg.edge_num = cubegraph->GetGraphUndirectedEdges().size();
//////		for(auto& eg : cubegraph->GetGraphUndirectedEdges())
//////		{
//////			srcl_lcm_msgs::Edge_t edge;
//////			edge.id_start = eg.src_->vertex_id_;
//////			edge.id_end = eg.dst_->vertex_id_;
//////
//////			graph_msg.edges.push_back(edge);
//////		}
////
////		lcm_->publish("quad_planner/geo_mark_graph", &graph_msg);
////
////		std::cout << "######################## graph sent ########################" << std::endl;
////	}
//}

void QuadPathRepair::LcmOctomapHandler(
		const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const srcl_lcm_msgs::NewDataReady_t* msg)
{
	std::cout << "\n---------------------- New Iteration -------------------------" << std::endl;

	LOG(INFO) << "----------- New iteration -----------";

	if(mission_tracker_->remaining_path_length_ < 0.5)
	{
		std::cout << "Getting close to goal, no need to replan" << std::endl;
		LOG(INFO) << "Getting close to goal, no need to replan";
		return;
	}

	static int count = 0;
	srcl_lcm_msgs::KeyframeSet_t kf_cmd;

	// record the planning time
	kf_cmd.sys_time.time_stamp = current_sys_time_;

	std::vector<Position3Dd> raw_wps;
	std::vector<GeoMark> geo_path;

	//std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctree(octomap_server_.octree_);
	std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctreeWithExtObstacle(octomap_server_.octree_);
	std::shared_ptr<Graph<CubeCell&>> cubegraph = GraphBuilder::BuildFromCubeArray(cubearray);

	// don't replan if 3d information is too limited
	if(mission_tracker_->mission_started_ && (cubearray->cubes_.size() == 0 || cubegraph->GetGraphVertices().size() < 5))
	{
		std::cerr << "Too limited 3D information collected" << std::endl;
		LOG(INFO) << "Too limited 3D information collected";
		return;
	}

	int64_t geo_start_id_astar = geomark_graph_.MergeCubeArrayInfo(cubegraph, cubearray);

	// don't replan if failed to combine graphs
	if(geo_start_id_astar == -1)
	{
		std::cerr << "Failed to combine graphs" << std::endl;
		LOG(INFO) << "Failed to combine graphs";
		return;
	}

	LOG(INFO) << "Combined graph size: " << sgrid_planner_.graph_->GetGraphVertices().size();
	std::cout << "Graph size (combined, 3d): " << sgrid_planner_.graph_->GetGraphVertices().size() << " , " << cubegraph->GetGraphVertices().size() << std::endl;

	uint64_t map_goal_id = sgrid_planner_.map_.data_model->GetIDFromPosition(goal_pos_.x, goal_pos_.y);
	uint64_t geo_goal_id_astar = sgrid_planner_.graph_->GetVertexFromID(map_goal_id)->bundled_data_->geo_mark_id_;

	clock_t exec_time;
	exec_time = clock();
	//auto path = AStar::Search(geomark_graph_.combined_graph_, geo_start_id_astar, geo_goal_id_astar);
	auto path = AStar::BiasedSearchWithShortcut(geomark_graph_.combined_graph_, geo_start_id_astar, geo_goal_id_astar, nav_field_->max_rewards_, sc_evaluator_->dist_weight_, sgrid_planner_.map_.data_model->cell_size_);
	exec_time = clock() - exec_time;
	std::cout << "Search in 3D finished in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

	for(auto& wp:path) {
		geo_path.push_back(wp->bundled_data_);
		raw_wps.push_back(wp->bundled_data_.position);
	}

	// if failed to find a 3d path, terminate this iteration
	if(raw_wps.size() <= 1)
		return;

	std::vector<Position3Dd> selected_wps = raw_wps;// MissionUtils::GetKeyTurningWaypoints(raw_wps);

	if(!mission_tracker_->mission_started_ || EvaluateNewPath(selected_wps))
	{
		if(mission_tracker_->mission_started_)
			std::cout << "-------- found better solution ---------" << std::endl;
		else
			mission_tracker_->mission_started_ = true;

		// update mission tracking information
		mission_tracker_->UpdateActivePathWaypoints(geo_path);
		mission_tracker_->remaining_path_length_ = est_new_dist_;

		kf_cmd.path_id = mission_tracker_->path_id_;
		kf_cmd.kf_num = selected_wps.size();
		double last_yaw = 0;
		for(auto& wp:selected_wps)
		{
			srcl_lcm_msgs::Keyframe_t kf;
			kf.vel_constr = false;

			kf.positions[0] = wp.x;
			kf.positions[1] = wp.y;
			kf.positions[2] = wp.z;

			uint64_t nd_id = sgrid_planner_.map_.data_model->GetIDFromPosition(wp.x, wp.y);
			auto nd_vtx = nav_field_->field_graph_->GetVertexFromID(nd_id);
			if(nd_vtx != nullptr) {
				kf.yaw = nd_vtx->rewards_yaw_;

				if(kf.yaw != 0)
					last_yaw = kf.yaw;
			}
			else
				kf.yaw = last_yaw;

			LOG(INFO) << "way point yaw: " << kf.yaw << " at id: " << nd_id;

			kf_cmd.kfs.push_back(kf);

		}
		//kf_cmd.kfs.front().yaw = 0;
		//kf_cmd.kfs.back().yaw = -M_PI/4;

		lcm_->publish("quad_planner/goal_keyframe_set", &kf_cmd);

		// send data for visualization
		Send3DSearchPathToVis(selected_wps);
	}

//		if(count++ % 20 == 0)
//		{
//			Send3DSearchPathToVis(selected_wps);
//
//			srcl_lcm_msgs::Graph_t graph_msg;
//
//			// combined graph
//			graph_msg.vertex_num = geomark_graph_.combined_graph_base_.GetGraphVertices().size();
//			for(auto& vtx : geomark_graph_.combined_graph_base_.GetGraphVertices())
//			{
//				srcl_lcm_msgs::Vertex_t vertex;
//				vertex.id = vtx->vertex_id_;
//
//				vertex.position[0] = vtx->bundled_data_.position.x;
//				vertex.position[1] = vtx->bundled_data_.position.y;
//				vertex.position[2] = vtx->bundled_data_.position.z;
//
//				graph_msg.vertices.push_back(vertex);
//			}
//
//			graph_msg.edge_num = geomark_graph_.combined_graph_base_.GetGraphUndirectedEdges().size();
//			for(auto& eg : geomark_graph_.combined_graph_base_.GetGraphUndirectedEdges())
//			{
//				srcl_lcm_msgs::Edge_t edge;
//				edge.id_start = eg.src_->vertex_id_;
//				edge.id_end = eg.dst_->vertex_id_;
//
//				graph_msg.edges.push_back(edge);
//			}

			// cube graph
	//		graph_msg.vertex_num = cubegraph->GetGraphVertices().size();
	//		for(auto& vtx : cubegraph->GetGraphVertices())
	//		{
	//			srcl_lcm_msgs::Vertex_t vertex;
	//			vertex.id = vtx->vertex_id_;
	//
	//			vertex.position[0] = vtx->bundled_data_.location_.x;
	//			vertex.position[1] = vtx->bundled_data_.location_.y;
	//			vertex.position[2] = vtx->bundled_data_.location_.z;
	//
	//			graph_msg.vertices.push_back(vertex);
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

			//lcm_->publish("quad_planner/geo_mark_graph", &graph_msg);
//
//			std::cout << "######################## graph sent ########################" << std::endl;
//		}
}

template<typename PlannerType>
srcl_lcm_msgs::Graph_t QuadPathRepair::GetLcmGraphFromPlanner(const PlannerType& planner)
{
	srcl_lcm_msgs::Graph_t graph_msg;

	graph_msg.vertex_num = planner.graph_->GetGraphVertices().size();
	for(auto& vtx : planner.graph_->GetGraphVertices())
	{
		srcl_lcm_msgs::Vertex_t vertex;
		vertex.id = vtx->vertex_id_;

		Position2Dd ref_world_pos = MapUtils::CoordinatesFromMapPaddedToRefWorld(vtx->bundled_data_->location_, planner.map_.info);
		vertex.position[0] = ref_world_pos.x;
		vertex.position[1] = ref_world_pos.y;

		graph_msg.vertices.push_back(vertex);
	}

	graph_msg.edge_num = planner.graph_->GetGraphUndirectedEdges().size();
	for(auto& eg : planner.graph_->GetGraphUndirectedEdges())
	{
		srcl_lcm_msgs::Edge_t edge;
		edge.id_start = eg.src_->vertex_id_;
		edge.id_end = eg.dst_->vertex_id_;

		graph_msg.edges.push_back(edge);
	}

	return graph_msg;
}

srcl_lcm_msgs::Graph_t QuadPathRepair::GenerateLcmGraphMsg()
{
	srcl_lcm_msgs::Graph_t graph_msg;

	graph_msg = GetLcmGraphFromPlanner(this->sgrid_planner_);

	return graph_msg;
}

srcl_lcm_msgs::Path_t QuadPathRepair::GenerateLcmPathMsg(std::vector<Position2D> waypoints)
{
	srcl_lcm_msgs::Path_t path_msg;

	path_msg.waypoint_num = waypoints.size();
	for(auto& wp : waypoints)
	{
		Position2Dd ref_world_pos = MapUtils::CoordinatesFromMapPaddedToRefWorld(wp, this->sgrid_planner_.map_.info);
		srcl_lcm_msgs::WayPoint_t waypoint;
		waypoint.positions[0] = ref_world_pos.x;
		waypoint.positions[1] = ref_world_pos.y;

		path_msg.waypoints.push_back(waypoint);
	}

	return path_msg;
}

void QuadPathRepair::Send3DSearchPathToVis(std::vector<Position3Dd> path)
{
	if(path.size() > 0)
	{
		srcl_lcm_msgs::Path_t path_msg;

		path_msg.waypoint_num = path.size();
		for(auto& wp : path)
		{
			srcl_lcm_msgs::WayPoint_t waypoint;
			waypoint.positions[0] = wp.x;
			waypoint.positions[1] = wp.y;
			waypoint.positions[2] = wp.z;

			path_msg.waypoints.push_back(waypoint);
		}

		lcm_->publish("quad_planner/geo_mark_graph_path", &path_msg);
	}
}
