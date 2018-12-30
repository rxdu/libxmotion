/*
 * sim_path_repair.cpp
 *
 *  Created on: Sep 9, 2017
 *      Author: rdu
 */

#include <iostream>
#include <ctime>
#include <cmath>
#include <limits>

#include "eigen3/Eigen/Geometry"

#include "logging/loggers.hpp"

#include "path_repair/sim/sim_path_repair.h"
#include "map/map_utils.h"

#include "map/cube_array.h"
#include "map/cube_array_builder.h"
#include "map/graph_builder.h"

#include "vis/sgrid_vis.h"
#include "vis/graph_vis.h"

using namespace librav;

// #define MINIMAL_EXTRAS

SimPathRepair::SimPathRepair(std::shared_ptr<lcm::LCM> lcm) : lcm_(lcm),
															  map_received_(false),
															  update_global_plan_(false),
															  mission_tracker_(new MissionTracker(lcm_)),
															  sensor_range_(5.0),
															  path_2d_cost_(std::numeric_limits<double>::infinity()),
															  path_3d_cost_(std::numeric_limits<double>::infinity()),
															  pstart_set_(false),
															  pgoal_set_(false),
															  hstart_set_(false),
															  hgoal_set_(false),
															  est_new_dist_(std::numeric_limits<double>::infinity())
{
	// default map size
	map_size_[0] = 5;
	map_size_[1] = 5;
	map_size_[2] = 5;

	// lcm subscription
	lcm_->subscribe("envsim/map", &SimPathRepair::LcmSimMapHandler, this);
}

SimPathRepair::SimPathRepair(std::shared_ptr<lcm::LCM> lcm, std::shared_ptr<SimDepthSensor> dsensor) : lcm_(lcm),
																									   depth_sensor_(dsensor),
																									   map_received_(false),
																									   update_global_plan_(false),
																									   mission_tracker_(new MissionTracker(lcm_)),
																									   sensor_range_(5.0),
																									   path_2d_cost_(std::numeric_limits<double>::infinity()),
																									   path_3d_cost_(std::numeric_limits<double>::infinity()),
																									   pstart_set_(false),
																									   pgoal_set_(false),
																									   hstart_set_(false),
																									   hgoal_set_(false),
																									   est_new_dist_(std::numeric_limits<double>::infinity())
{
	// default map size
	map_size_[0] = 5;
	map_size_[1] = 5;
	map_size_[2] = 5;

	// lcm subscription
	lcm_->subscribe("envsim/map", &SimPathRepair::LcmSimMapHandler, this);
}

void SimPathRepair::SetSensorRange(int32_t rng)
{
	sensor_range_ = rng;
	if (depth_sensor_ != nullptr)
		depth_sensor_->SetRange(sensor_range_);
}

bool SimPathRepair::IsConfigComplete()
{
	if (pstart_set_ && pgoal_set_ && hstart_set_ && hgoal_set_)
		return true;
	else
		return false;
}

void SimPathRepair::SetStartHeight(int32_t height)
{
	start_height_ = height;
	hstart_set_ = true;
}

void SimPathRepair::SetGoalHeight(int32_t height)
{
	goal_height_ = height;
	hgoal_set_ = true;
}

void SimPathRepair::SetMapSize(int32_t x, int32_t y, int32_t z)
{
	map_size_[0] = x;
	map_size_[1] = y;
	map_size_[2] = z;
}

void SimPathRepair::SetStartPosition(Position2Di pos)
{
	if (pstart_set_ && pos == start_pos_)
		return;

	start_pos_.x = pos.x;
	start_pos_.y = pos.y;

	pstart_set_ = true;
}

void SimPathRepair::SetGoalPosition(Position2Di pos)
{
	goal_pos_.x = pos.x;
	goal_pos_.y = pos.y;

	pgoal_set_ = true;
}

std::vector<uint64_t> SimPathRepair::UpdateGlobal2DPath()
{
	std::vector<uint64_t> waypoints;

	auto start_id = sgrid_->GetIDFromIndex(start_pos_.x, start_pos_.y);
	auto goal_id = sgrid_->GetIDFromIndex(goal_pos_.x, goal_pos_.y);

	std::cout << "sgrid size: " << sgrid_->row_size_ << " , " << sgrid_->col_size_ << std::endl;

	std::cout << "----> start: " << start_pos_.x << " , " << start_pos_.y << ", id: " << start_id << std::endl;
	std::cout << "----> goal: " << goal_pos_.x << " , " << goal_pos_.y << ", id: " << goal_id << std::endl;
	//std::cout << "col: " << sgrid_->col_size_ << " , row: " << sgrid_->row_size_ << std::endl;

	auto traj_vtx = sgrid_planner_.Search(start_id, goal_id);

	if (!traj_vtx.empty())
	{
		path_2d_cost_ = traj_vtx.back()->GetAStarGCost();
		std::cout << "** shortest 2D path cost: " << path_2d_cost_ << std::endl;
	}

	for (auto &wp : traj_vtx)
		waypoints.push_back(wp->vertex_id_);

	update_global_plan_ = false;

	return waypoints;
}

void SimPathRepair::ResetPlanner()
{
	est_new_dist_ = std::numeric_limits<double>::infinity();
	map_received_ = false;
}

void SimPathRepair::RequestNewMap()
{
	librav_lcm_msgs::MapRequest_t map_rqt_msg;
	map_rqt_msg.new_map_requested = true;
	map_rqt_msg.map_size_x = map_size_[0];
	map_rqt_msg.map_size_y = map_size_[1];
	map_rqt_msg.map_size_z = map_size_[2];
	map_rqt_msg.map_type = 2;
	lcm_->publish("envsim/map_request", &map_rqt_msg);
	std::cout << "New map request sent" << std::endl;
}

void SimPathRepair::SaveMap(std::string map_name)
{
	auto vis_grid = MapUtils::CreateSquareGrid(sgrid_->col_size_, sgrid_->row_size_, 50);
	for (auto &cell : sgrid_->cells_)
	{
		if (cell.second->occu_ == OccupancyType::OCCUPIED)
			vis_grid->SetCellOccupancy(cell.second->index_.x, cell.second->index_.y, OccupancyType::OCCUPIED);
	}

	cv::Mat vis_img;
	Vis::VisSquareGrid(*vis_grid, vis_img);
	cv::imwrite(map_name + ".png", vis_img);

	std::shared_ptr<Graph_t<SquareCell *>> vis_graph = GraphBuilder::BuildFromSquareGrid(vis_grid, true);
	std::shared_ptr<NavField<SquareCell *>> vis_nav_field = std::make_shared<NavField<SquareCell *>>(vis_graph);
	vis_nav_field->UpdateNavField(589);

	ShortcutEval sc_eval(vis_grid, vis_nav_field);
	sc_eval.EvaluateGridShortcutPotential(5);

	Vis::VisSquareGridShortcutPotential(*vis_nav_field, vis_img, vis_img);
	cv::imwrite(map_name+"_nav_field.png", vis_img);
}

void SimPathRepair::LcmSimMapHandler(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::Map_t *msg)
{
	// discard msg if a valid map has been received
	if (map_received_)
		return;

	// otherwise process the msg to get a new map
	std::cout << "Map msg received: " << std::endl;
	std::cout << "Map size: " << msg->cell_num << std::endl;

	// create square grid from map msg
	double side_size = 1.0;
	sgrid_ = MapUtils::CreateSquareGrid(msg->size_x, msg->size_y, side_size);
	// auto new_grid = MapUtils::CreateSquareGrid(msg->size_x, msg->size_y, side_size);
	// sgrid_ = SGridBuilderV2::BuildExtSquareGrid(new_grid, 1);
	carray_base_ = CubeArrayBuilder::BuildSolidCubeArray(msg->size_x, msg->size_y, msg->size_z, side_size);
	for (const auto &cell : msg->cells)
	{
		if (cell.occupied)
		{
			sgrid_->SetCellOccupancy(cell.pos_x, cell.pos_y, OccupancyType::OCCUPIED);
		}
		else
		{
			for (int i = 0; i < msg->size_z; i++)
				carray_base_->SetCubeOccupancy(cell.pos_x, cell.pos_y, i, OccupancyType::FREE);
		}
	}

	// set square grid to graph planner, and check if a path exists
	// 	if not request a new map
	bool result = sgrid_planner_.UpdateMapConfig(sgrid_);
	auto path = UpdateGlobal2DPath();
	if (!path.empty())
	{
		std::cout << "Validate map received" << std::endl;

		// init and update navigation field and shortcut evaluator
		nav_field_ = std::make_shared<NavField<SquareCell *>>(sgrid_planner_.graph_);
		sc_evaluator_ = std::make_shared<ShortcutEval>(sgrid_, nav_field_);

		auto goal_id = sgrid_->GetIDFromIndex(goal_pos_.x, goal_pos_.y);
		nav_field_->UpdateNavField(goal_id);

		sc_evaluator_->EvaluateGridShortcutPotential(sensor_range_);

		if (nav_field_->max_rewards_ > path_2d_cost_ * 0.20)
		{
			// update flags
			map_received_ = true;
			update_global_plan_ = true;

			// update info of virtual space for 3d planning
			map_info_.size_x = msg->size_x;
			map_info_.size_y = msg->size_y;
			map_info_.size_z = msg->size_z;
			map_info_.side_size = side_size;

			depth_sensor_->SetWorkspace(msg, side_size);
			std::cout << "Configs for new map updated" << std::endl;
		}
		else
		{
			std::cout << "Invalid map" << std::endl;
			RequestNewMap();
		}
	}
	else
	{
		std::cout << "Invalid map" << std::endl;
		RequestNewMap();
	}

	// cv::Mat vis_img;
	// Vis::VisSquareGrid(*sgrid_, vis_img);
	// Vis::VisGraph(*sgrid_planner_.graph_, vis_img, vis_img, true);
	// cv::namedWindow("Processed Image", cv::WINDOW_NORMAL);
	// cv::imshow("Processed Image", vis_img);
	// cv::waitKey(0);
}

bool SimPathRepair::EvaluateNewPath(std::vector<Position3Dd> &new_path)
{
	if (std::sqrt(std::pow(new_path.front().x - mission_tracker_->current_position_.x, 2) +
				  std::pow(new_path.front().y - mission_tracker_->current_position_.y, 2) +
				  std::pow(new_path.front().z - mission_tracker_->current_position_.z, 2)) > 0.35)
	{
		std::cout << "rejected plan due to wrong starting point" << std::endl;
		return false;
	}

	est_new_dist_ = 0;
	for (int i = 0; i < new_path.size() - 1; i++)
		est_new_dist_ += std::sqrt(std::pow(new_path[i].x - new_path[i + 1].x, 2) +
								   std::pow(new_path[i].y - new_path[i + 1].y, 2)); // +
																					//std::pow(new_path[i].z - new_path[i + 1].z,2));

	std::cout << "new path dist: " << est_new_dist_ << " , remaining dist of current path: "
			  << mission_tracker_->remaining_path_length_ << std::endl;

	// LOG(INFO) << "old_dist = " <<  mission_tracker_->remaining_path_length_
	// 				<< " , new_dist = " << est_new_dist_;

	if (new_path.size() > 0 && est_new_dist_ < mission_tracker_->remaining_path_length_ * 0.85)
	{
		// LOG(INFO) << " --------> new plan found <-------- ";
		// LOG(INFO) << "remaining path length: " <<  mission_tracker_->remaining_path_length_
		// 					<< " , new path length: " << est_new_dist_;
		return true;
	}
	else
		return false;
}

SimPath SimPathRepair::UpdatePath(Position2Di pos, int32_t height, double heading, bool enable_path_repair)
{
	std::cout << "\n---------------------- New Iteration -------------------------" << std::endl;
	if(enable_path_repair)
		std::cout << "run type: repair" << std::endl;
	else
		std::cout << "run type: shortest" << std::endl;
	//auto path = UpdateGlobal2DPath();

	// create an empty cube array
	std::shared_ptr<CubeArray> carray = CubeArrayBuilder::BuildSolidCubeArray(map_info_.size_x, map_info_.size_y, map_info_.size_z, map_info_.side_size);

	// add 2d map info into cube array
	for (int j = 0; j < map_info_.size_y; j++)
		for (int i = 0; i < map_info_.size_x; i++)
		{
			auto id = carray->GetIDFromIndex(i, j, height);
			if (carray_base_->cubes_[id].occu_ == OccupancyType::FREE)
				carray->cubes_[id].occu_ = OccupancyType::FREE;
		}

	// add 3d info into cube array
	auto sensor_carray = depth_sensor_->GetSensedArea(pos.x, pos.y, height, heading);
	for (int k = 0; k < map_info_.size_z; k++)
		for (int j = 0; j < map_info_.size_y; j++)
			for (int i = 0; i < map_info_.size_x; i++)
			{
				auto id = carray->GetIDFromIndex(i, j, k);
				if (sensor_carray->cubes_[id].occu_ == OccupancyType::FREE)
					carray->cubes_[id].occu_ = OccupancyType::FREE;
			}

	// create a graph from the cube array
	std::shared_ptr<Graph_t<CubeCell &>> cubegraph = GraphBuilder::BuildFromCubeArray(carray);
// #ifndef MINIMAL_EXTRAS
// 	std::cout << "cube array size: " << carray->cubes_.size() << std::endl;
// 	std::cout << "cube graph size: " << cubegraph->GetGraphVertices().size() << " , edge num: " << cubegraph->GetGraphEdges().size() << std::endl;
// #endif
	auto start_id = carray->GetIDFromIndex(pos.x, pos.y, height);
	auto goal_id = carray->GetIDFromIndex(goal_pos_.x, goal_pos_.y, goal_height_);
// #ifndef MINIMAL_EXTRAS
// 	std::cout << "start id: " << start_id << " , goal id: " << goal_id << std::endl;
// 	std::cout << "heading: " << heading * 180.0 / M_PI << std::endl;

// 	std::cout << "max rewards: " << nav_field_->max_rewards_ << " , dist weight: " << sc_evaluator_->dist_weight_ << std::endl;
// #endif

	Path_t<CubeCell &> path;
	if(enable_path_repair)
		path = AStar::BiasedSearchWithShortcut(*cubegraph, start_id, goal_id, nav_field_->max_rewards_, sc_evaluator_->dist_weight_, map_info_.side_size);
	else
		path = AStar::Search(*cubegraph, start_id, goal_id);

	SimPath path_result;

	if (!path.empty())
	{
		path_3d_cost_ = path.back()->GetAStarGCost();

		librav_lcm_msgs::GridPath_t path_msg;
		path_msg.pt_num = path.size();
		// double prev_heading = 0;
		for (auto &cell : path)
		{
			double heading = 0;
			uint64_t nd_id = sgrid_->GetIDFromPosition(cell->bundled_data_.index_.x, cell->bundled_data_.index_.y);
			auto nd_vtx = nav_field_->field_graph_->GetVertexFromID(nd_id);
			if (nd_vtx != nullptr)
			{
				heading = nd_vtx->rewards_yaw_ / 180.0 * M_PI;

				// if (heading != 0)
				// 	prev_heading = heading;
			}
			else
			{
				heading = 0;
				// heading = prev_heading;
			}

			librav_lcm_msgs::GridWaypoint_t wp;
			wp.id = cell->bundled_data_.data_id_;
			wp.x = cell->bundled_data_.index_.x;
			wp.y = cell->bundled_data_.index_.y;
			wp.z = cell->bundled_data_.index_.z;
			wp.yaw = heading;

			path_msg.points.push_back(wp);

			SimWaypoint swp;
			swp.id = cell->bundled_data_.data_id_;
			swp.x = cell->bundled_data_.index_.x;
			swp.y = cell->bundled_data_.index_.y;
			swp.z = cell->bundled_data_.index_.z;
			swp.yaw = heading;
			path_result.push_back(swp);
		}
		lcm_->publish("quad_planner/repaired_path", &path_msg);

#ifndef MINIMAL_EXTRAS
		SendCubeArrayGraphToVis(cubegraph);
		Send3DSearchPathToVis(path);
#endif
	}
	else
	{
		std::cout << "no path found" << std::endl;
	}

	return path_result;
}

void SimPathRepair::Send3DSearchPathToVis(Path_t<CubeCell &> &path)
{
	if (path.size() > 0)
	{
		librav_lcm_msgs::GridPath_t path_msg;

		path_msg.waypoint_num = path.size();
		for (auto &wp : path)
		{
			librav_lcm_msgs::WayPoint_t waypoint;
			waypoint.positions[0] = wp->bundled_data_.location_.x;
			waypoint.positions[1] = wp->bundled_data_.location_.y;
			waypoint.positions[2] = wp->bundled_data_.location_.z;

			path_msg.waypoints.push_back(waypoint);
		}

		lcm_->publish("quad_planner/geo_mark_graph_path", &path_msg);
	}
}

void SimPathRepair::SendCubeArrayGraphToVis(std::shared_ptr<Graph_t<CubeCell &>> cubegraph)
{
	// cube graph
	librav_lcm_msgs::Graph_t graph_msg;

	graph_msg.vertex_num = cubegraph->GetGraphVertices().size();
	for (auto &vtx : cubegraph->GetGraphVertices())
	{
		librav_lcm_msgs::Vertex_t vertex;
		vertex.id = vtx->vertex_id_;

		vertex.position[0] = vtx->bundled_data_.location_.x;
		vertex.position[1] = vtx->bundled_data_.location_.y;
		vertex.position[2] = vtx->bundled_data_.location_.z;
		graph_msg.vertices.push_back(vertex);
	}

	graph_msg.edge_num = cubegraph->GetGraphUndirectedEdges().size();
	for (auto &eg : cubegraph->GetGraphUndirectedEdges())
	{
		librav_lcm_msgs::Edge_t edge;
		edge.id_start = eg.src_->vertex_id_;
		edge.id_end = eg.dst_->vertex_id_;

		graph_msg.edges.push_back(edge);
	}

	//std::cout << "vertex size: " << graph_msg.vertex_num << " , edge size: " << graph_msg.edge_num << std::endl;

	lcm_->publish("quad_planner/geo_mark_graph", &graph_msg);

	std::cout << "######################## graph sent ########################" << std::endl;
}
