/*
 * quad_path_repair.cpp
 *
 *  Created on: Sep 9, 2016
 *      Author: rdu
 */

#include <iostream>
#include <ctime>

#include "path_repair/quad_path_repair.h"
#include "map/map_utils.h"
#include "geometry/cube_array/cube_array.h"
#include "geometry/cube_array_builder.h"
#include "geometry/graph_builder.h"

using namespace srcl_ctrl;

QuadPathRepair::QuadPathRepair(std::shared_ptr<lcm::LCM> lcm):
		lcm_(lcm),
		octomap_server_(OctomapServer(lcm_)),
		motion_client_(PolyMotionClient(lcm,"quad_planner/polynomial_curve", "quad_controller/quad_motion_service")),
		active_graph_planner_(GraphPlannerType::NOT_SPECIFIED),
		gstart_set_(false),
		ggoal_set_(false),
		world_size_set_(false),
		auto_update_pos_(true),
		update_global_plan_(false)
{
	if(!lcm_->good())
		std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
	else {
		lcm_->subscribe("quad_data/quad_transform",&QuadPathRepair::LcmTransformHandler, this);
		lcm_->subscribe("quad_planner/new_octomap_ready",&QuadPathRepair::LcmOctomapHandler, this);
	}
}

QuadPathRepair::~QuadPathRepair()
{

}

void QuadPathRepair::ConfigGraphPlanner(MapConfig config, double world_size_x, double world_size_y)
{
	if(config.GetMapType().data_model == MapDataModel::QUAD_TREE)
	{
		bool result = qtree_planner_.UpdateMapConfig(config);

		if(result)
		{
			std::cout << "quad tree planner activated" << std::endl;
			active_graph_planner_ = GraphPlannerType::QUADTREE_PLANNER;

			//gcombiner_.SetBaseGraph(qtree_planner_.graph_, qtree_planner_.map_.data_model, qtree_planner_.map_.data_model->leaf_nodes_.size(), qtree_planner_.map_.info);
		}
	}
	else if(config.GetMapType().data_model == MapDataModel::SQUARE_GRID)
	{
		bool result = sgrid_planner_.UpdateMapConfig(config);

		if(result)
		{
			std::cout << "square grid planner activated" << std::endl;
			active_graph_planner_ = GraphPlannerType::SQUAREGRID_PLANNER;
		}
	}

	// the world size must be set after the planner is updated, otherwise the configuration will be override
	qtree_planner_.map_.info.SetWorldSize(world_size_x, world_size_y);
	sgrid_planner_.map_.info.SetWorldSize(world_size_x, world_size_y);
	world_size_set_ = true;

	if(active_graph_planner_ == GraphPlannerType::SQUAREGRID_PLANNER)
		gcombiner_.SetBaseGraph(sgrid_planner_.graph_, sgrid_planner_.map_.data_model, sgrid_planner_.map_.data_model->cells_.size(), sgrid_planner_.map_.info);

	srcl_msgs::Graph_t graph_msg = GenerateLcmGraphMsg();
	lcm_->publish("quad_planner/quad_planner_graph", &graph_msg);
}

void QuadPathRepair::SetStartMapPosition(Position2D pos)
{
	if(pos == start_pos_)
		return;

	start_pos_.x = pos.x;
	start_pos_.y = pos.y;

	gstart_set_ = true;

	if(gstart_set_ && ggoal_set_)
		update_global_plan_ = true;
}

void QuadPathRepair::SetGoalMapPosition(Position2D pos)
{
	goal_pos_.x = pos.x;
	goal_pos_.y = pos.y;

	ggoal_set_ = true;

	if(gstart_set_ && ggoal_set_)
		update_global_plan_ = true;
}

void QuadPathRepair::SetStartRefWorldPosition(Position2Dd pos)
{
	Position2Dd mpos;
	mpos = MapUtils::CoordinatesFromRefWorldToMapWorld(pos, GetActiveMapInfo());

	Position2D map_pos;
	map_pos = MapUtils::CoordinatesFromMapWorldToMap(mpos, GetActiveMapInfo());

	Position2D map_padded_pos;
	map_padded_pos = MapUtils::CoordinatesFromOriginalToPadded(map_pos, GetActiveMapInfo());

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

std::vector<Position2D> QuadPathRepair::SearchForGlobalPath()
{
	std::vector<Position2D> waypoints;

//	std::cout << "----> start: " << start_pos_.x << " , " << start_pos_.y << std::endl;
//	std::cout << "----> goal: " << goal_pos_.x << " , " << goal_pos_.y << std::endl;

	if(active_graph_planner_ == GraphPlannerType::QUADTREE_PLANNER)
	{
		auto traj_vtx = qtree_planner_.Search(start_pos_, goal_pos_);
		for(auto& wp:traj_vtx)
			waypoints.push_back(wp->bundled_data_->location_);
	}
	else if(active_graph_planner_ == GraphPlannerType::SQUAREGRID_PLANNER)
	{
		auto traj_vtx = sgrid_planner_.Search(start_pos_, goal_pos_);
		for(auto& wp:traj_vtx)
			waypoints.push_back(wp->bundled_data_->location_);
	}

	srcl_msgs::Path_t path_msg = GenerateLcmPathMsg(waypoints);
	lcm_->publish("quad_planner/quad_planner_graph_path", &path_msg);

	update_global_plan_ = false;

	return waypoints;
}

std::vector<uint64_t> QuadPathRepair::SearchForGlobalPathID()
{
	std::vector<uint64_t> waypoints;

//		std::cout << "----> start: " << start_pos_.x << " , " << start_pos_.y << std::endl;
//		std::cout << "----> goal: " << goal_pos_.x << " , " << goal_pos_.y << std::endl;

	if(active_graph_planner_ == GraphPlannerType::QUADTREE_PLANNER)
	{
		auto traj_vtx = qtree_planner_.Search(start_pos_, goal_pos_);
		for(auto& wp:traj_vtx)
			waypoints.push_back(wp->vertex_id_);
	}
	else if(active_graph_planner_ == GraphPlannerType::SQUAREGRID_PLANNER)
	{
		auto traj_vtx = sgrid_planner_.Search(start_pos_, goal_pos_);
		for(auto& wp:traj_vtx)
			waypoints.push_back(wp->vertex_id_);
	}

	update_global_plan_ = false;

	return waypoints;
}

cv::Mat QuadPathRepair::GetActiveMap()
{
	if(active_graph_planner_ == GraphPlannerType::QUADTREE_PLANNER)
	{
		return qtree_planner_.map_.padded_image;
	}
	else if(active_graph_planner_ == GraphPlannerType::SQUAREGRID_PLANNER)
	{
		return sgrid_planner_.map_.padded_image;
	}
	else
		return cv::Mat::zeros(10, 10, CV_8UC1);
}

MapInfo QuadPathRepair::GetActiveMapInfo()
{
	MapInfo empty_info;

	if(active_graph_planner_ == GraphPlannerType::QUADTREE_PLANNER)
	{
		return qtree_planner_.map_.info;
	}
	else if(active_graph_planner_ == GraphPlannerType::SQUAREGRID_PLANNER)
	{
		return sgrid_planner_.map_.info;
	}
	else
		return empty_info;
}

void QuadPathRepair::LcmTransformHandler(
		const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const srcl_msgs::QuadrotorTransform* msg)
{
	Position2Dd rpos;
	rpos.x = msg->base_to_world.position[0];
	rpos.y = msg->base_to_world.position[1];

//	Position2Dd mapw_pos;
//	mapw_pos = MapUtils::CoordinatesFromRefWorldToMapWorld(rpos, GetActiveMapInfo());
//
//	Position2D map_pos;
//	map_pos = MapUtils::CoordinatesFromMapWorldToMap(mapw_pos, GetActiveMapInfo());
//
//	Position2D map_padded_pos;
//	map_padded_pos = MapUtils::CoordinatesFromOriginalToPadded(map_pos, GetActiveMapInfo());

//	std::cout << "quadrotor position in sim: " << msg->base_to_world.position[0] << " , "
//			<< msg->base_to_world.position[1] << " , "
//			<< msg->base_to_world.position[2] << std::endl;
//	std::cout << "quadrotor position in image world: " << mpos.x << " , "
//				<< mpos.y << std::endl;

	if(auto_update_pos_)
		SetStartRefWorldPosition(rpos);

	gcombiner_.UpdateFlightHeight(Position3Dd(msg->base_to_world.position[0],msg->base_to_world.position[1],msg->base_to_world.position[2]),
					Eigen::Quaterniond(msg->base_to_world.quaternion[0] , msg->base_to_world.quaternion[1] , msg->base_to_world.quaternion[2] , msg->base_to_world.quaternion[3]));
}

void QuadPathRepair::LcmOctomapHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const srcl_msgs::NewDataReady_t* msg)
{
	static int count = 0;
	std::cout << " test reading: " << octomap_server_.octree_->getResolution() << std::endl;

	std::shared_ptr<CubeArray> cubearray = CubeArrayBuilder::BuildCubeArrayFromOctree(octomap_server_.octree_);
	std::shared_ptr<Graph<CubeCell&>> cubegraph = GraphBuilder::BuildFromCubeArray(cubearray);

	bool combine_success = gcombiner_.CombineBaseWithCubeArrayGraph(cubearray, cubegraph);

	if(!combine_success)
		return;

	uint64_t map_start_id = sgrid_planner_.map_.data_model->GetIDFromPosition(start_pos_.x, start_pos_.y);
	uint64_t map_goal_id = sgrid_planner_.map_.data_model->GetIDFromPosition(goal_pos_.x, goal_pos_.y);;

	uint64_t geo_start_id_astar = sgrid_planner_.graph_->GetVertexFromID(map_start_id)->bundled_data_->geo_mark_id_;
	uint64_t geo_goal_id_astar = sgrid_planner_.graph_->GetVertexFromID(map_goal_id)->bundled_data_->geo_mark_id_;

	clock_t exec_time;
	exec_time = clock();
	auto comb_path = gcombiner_.combined_graph_.AStarSearch(geo_start_id_astar, geo_goal_id_astar);
	exec_time = clock() - exec_time;
	std::cout << "Search in 3D finished in " << double(exec_time)/CLOCKS_PER_SEC << " s." << std::endl;

	std::vector<Position3Dd> comb_path_pos;
	for(auto& wp:comb_path)
		comb_path_pos.push_back(wp->bundled_data_.position);

	std::vector<Position3Dd> octomap_waypoints;
	//octomap_waypoints.push_back(comb_path_pos.front());
	uint16_t wp_idx = 0;
	for(auto& wp:comb_path)
	{
//		if(wp->bundled_data_.source == GeoMarkSource::LASER_OCTOMAP)
//			octomap_waypoints.push_back(wp->bundled_data_.position);
		if(wp_idx++ == 0)
			continue;

		octomap_waypoints.push_back(wp->bundled_data_.position);

		if(wp_idx++ > 5)
			break;
	}

	uint8_t kf_num = octomap_waypoints.size();

	if(kf_num < 2)
		return;

	traj_opt_.InitOptJointMatrices(kf_num);

	for(int i = 0; i < octomap_waypoints.size(); i++)
	{
		traj_opt_.keyframe_x_vals_(0,i) = octomap_waypoints[i].x;
		traj_opt_.keyframe_y_vals_(0,i) = octomap_waypoints[i].y;
		traj_opt_.keyframe_z_vals_(0,i) = octomap_waypoints[i].z;
		traj_opt_.keyframe_yaw_vals_(0,i) = 0;

		traj_opt_.keyframe_x_vals_(1,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_y_vals_(1,i) = std::numeric_limits<float>::infinity();
		traj_opt_.keyframe_z_vals_(1,i) = std::numeric_limits<float>::infinity();

		traj_opt_.keyframe_ts_(0,i) = i * 1.0;
	}

	traj_opt_.keyframe_x_vals_(1,0) = 0.0;
	traj_opt_.keyframe_y_vals_(1,0) = 0.0;
	traj_opt_.keyframe_z_vals_(1,0) = 0.0;

	traj_opt_.keyframe_x_vals_(1,kf_num - 1) = 0.0;
	traj_opt_.keyframe_y_vals_(1,kf_num - 1) = 0.0;
	traj_opt_.keyframe_z_vals_(1,kf_num - 1) = 0.0;

	traj_opt_.OptimizeFlatTrajJoint();

//	if(count++ == 20)
//	{
//		count = 0;
//		srcl_msgs::Graph_t graph_msg;
//
//		graph_msg.vertex_num = gcombiner_.combined_graph_.GetGraphVertices().size();
//		for(auto& vtx : gcombiner_.combined_graph_.GetGraphVertices())
//		{
//			srcl_msgs::Vertex_t vertex;
//			vertex.id = vtx->vertex_id_;
//
//			vertex.position[0] = vtx->bundled_data_.position.x;
//			vertex.position[1] = vtx->bundled_data_.position.y;
//			vertex.position[2] = vtx->bundled_data_.position.z;
//
//			graph_msg.vertices.push_back(vertex);
//		}
//
//		graph_msg.edge_num = gcombiner_.combined_graph_.GetGraphUndirectedEdges().size();
//		for(auto& eg : gcombiner_.combined_graph_.GetGraphUndirectedEdges())
//		{
//			srcl_msgs::Edge_t edge;
//			edge.id_start = eg.src_->vertex_id_;
//			edge.id_end = eg.dst_->vertex_id_;
//
//			graph_msg.edges.push_back(edge);
//		}
//
//		lcm_->publish("quad_planner/geo_mark_graph", &graph_msg);
//	}

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

	lcm_->publish("quad_planner/geo_mark_graph_path", &path_msg);

	srcl_msgs::PolynomialCurve_t poly_msg;

	poly_msg.seg_num = traj_opt_.flat_traj_.traj_segs_.size();
	for(auto& seg : traj_opt_.flat_traj_.traj_segs_)
	{
		srcl_msgs::PolyCurveSegment_t seg_msg;

		seg_msg.coeffsize_x = seg.seg_x.param_.coeffs.size();
		seg_msg.coeffsize_y = seg.seg_y.param_.coeffs.size();
		seg_msg.coeffsize_z = seg.seg_z.param_.coeffs.size();
		seg_msg.coeffsize_yaw = seg.seg_yaw.param_.coeffs.size();
		for(auto& coeff:seg.seg_x.param_.coeffs)
			seg_msg.coeffs_x.push_back(coeff);
		for(auto& coeff:seg.seg_y.param_.coeffs)
			seg_msg.coeffs_y.push_back(coeff);
		for(auto& coeff:seg.seg_z.param_.coeffs)
			seg_msg.coeffs_z.push_back(coeff);
		for(auto& coeff:seg.seg_yaw.param_.coeffs)
			seg_msg.coeffs_yaw.push_back(coeff);

		seg_msg.t_start = 0;
		seg_msg.t_end = 1.0;

		poly_msg.segments.push_back(seg_msg);
	}

	lcm_->publish("quad_planner/polynomial_curve", &poly_msg);
}

template<typename PlannerType>
srcl_msgs::Graph_t QuadPathRepair::GetLcmGraphFromPlanner(const PlannerType& planner)
{
	srcl_msgs::Graph_t graph_msg;

	graph_msg.vertex_num = planner.graph_->GetGraphVertices().size();
	for(auto& vtx : planner.graph_->GetGraphVertices())
	{
		srcl_msgs::Vertex_t vertex;
		vertex.id = vtx->vertex_id_;

		Position2Dd ref_world_pos = MapUtils::CoordinatesFromMapPaddedToRefWorld(vtx->bundled_data_->location_, planner.map_.info);
		vertex.position[0] = ref_world_pos.x;
		vertex.position[1] = ref_world_pos.y;

		graph_msg.vertices.push_back(vertex);
	}

	graph_msg.edge_num = planner.graph_->GetGraphUndirectedEdges().size();
	for(auto& eg : planner.graph_->GetGraphUndirectedEdges())
	{
		srcl_msgs::Edge_t edge;
		edge.id_start = eg.src_->vertex_id_;
		edge.id_end = eg.dst_->vertex_id_;

		graph_msg.edges.push_back(edge);
	}

	return graph_msg;
}

srcl_msgs::Graph_t QuadPathRepair::GenerateLcmGraphMsg()
{
	srcl_msgs::Graph_t graph_msg;

	if(active_graph_planner_ == GraphPlannerType::QUADTREE_PLANNER)
	{
		graph_msg = GetLcmGraphFromPlanner(this->qtree_planner_);
	}
	else if(active_graph_planner_ == GraphPlannerType::SQUAREGRID_PLANNER)
	{
		graph_msg = GetLcmGraphFromPlanner(this->sgrid_planner_);
	}

	return graph_msg;
}

srcl_msgs::Path_t QuadPathRepair::GenerateLcmPathMsg(std::vector<Position2D> waypoints)
{
	srcl_msgs::Path_t path_msg;

	if(active_graph_planner_ == GraphPlannerType::QUADTREE_PLANNER)
	{
		path_msg.waypoint_num = waypoints.size();
		for(auto& wp : waypoints)
		{
			Position2Dd ref_world_pos = MapUtils::CoordinatesFromMapPaddedToRefWorld(wp, this->qtree_planner_.map_.info);
			srcl_msgs::WayPoint_t waypoint;
			waypoint.positions[0] = ref_world_pos.x;
			waypoint.positions[1] = ref_world_pos.y;

			path_msg.waypoints.push_back(waypoint);
		}
	}
	else if(active_graph_planner_ == GraphPlannerType::SQUAREGRID_PLANNER)
	{
		path_msg.waypoint_num = waypoints.size();
		for(auto& wp : waypoints)
		{
			Position2Dd ref_world_pos = MapUtils::CoordinatesFromMapPaddedToRefWorld(wp, this->sgrid_planner_.map_.info);
			srcl_msgs::WayPoint_t waypoint;
			waypoint.positions[0] = ref_world_pos.x;
			waypoint.positions[1] = ref_world_pos.y;

			path_msg.waypoints.push_back(waypoint);
		}
	}

	return path_msg;
}
