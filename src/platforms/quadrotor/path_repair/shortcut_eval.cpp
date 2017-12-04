/*
 * shortcut_eval.cpp
 *
 *  Created on: Jan 31, 2017
 *      Author: rdu
 */

#include <cmath>
#include <queue>
#include <algorithm>
#include <unistd.h>

#include "eigen3/Eigen/Geometry"

#include "vis/graph_vis.h"
#include "vis/sgrid_vis.h"
#include "vis/vis_utils.h"
#include "planning/graph/priority_queue.h"
#include "path_repair/shortcut_eval.h"

using namespace librav;

ShortcutEval::ShortcutEval(std::shared_ptr<SquareGrid> sgrid, std::shared_ptr<NavField<SquareCell*>> nav_field):
			sgrid_(sgrid),
			nav_field_(nav_field),
			dist_weight_(0.8)
{

}

double ShortcutEval::CalcDirectDistance(Position2Di start, Position2Di goal, double cell_size, bool allow_diag)
{
	double dist = 0;

	int32_t x_error, y_error;

	if(start.x > goal.x)
		x_error = start.x - goal.x;
	else
		x_error = goal.x - start.x;

	if(start.y > goal.y)
		y_error = start.y - goal.y;
	else
		y_error = goal.y - start.y;

	if(!allow_diag) {
		dist = (x_error + y_error) * cell_size;
	}
	else {
		int32_t diag_steps = 0;
		int32_t straight_steps = 0;

		if(x_error > y_error) {
			diag_steps = y_error;
			straight_steps = x_error - y_error;
		}
		else {
			diag_steps = x_error;
			straight_steps = y_error - x_error;
		}

		dist = diag_steps * std::sqrt(2) * cell_size + straight_steps * cell_size;
	}

	return dist;
}

double ShortcutEval::EvaluateCellShortcutPotential(Vertex_t<SquareCell*>* eval_vtx, uint16_t sensor_range)
{
	auto nbs = sgrid_->GetNeighboursWithinRange(eval_vtx->bundled_data_->data_id_, sensor_range);

	double max_rwd;
	Vertex_t<SquareCell*>* max_rwd_vtx = nullptr;
	for(auto& n : nbs) {
		if(n->occu_ == OccupancyType::OCCUPIED || nav_field_->field_graph_->GetVertexFromID(n->data_id_) == nullptr)
			continue;

		auto vtx = nav_field_->field_graph_->GetVertexFromID(n->data_id_);

		double rewards = eval_vtx->potential_ - vtx->potential_
				- CalcDirectDistance(vtx->bundled_data_->index_, eval_vtx->bundled_data_->index_,sgrid_->cell_size_,true);

		if(max_rwd_vtx == nullptr || rewards > max_rwd) {
			max_rwd = rewards;
			max_rwd_vtx = vtx;
		}
	}

	// update heading angle
	double angle = 0;
	Eigen::Vector3d max_rwd_vec(max_rwd_vtx->bundled_data_->index_.x, max_rwd_vtx->bundled_data_->index_.y, 0);
	Eigen::Vector3d pos_vec(eval_vtx->bundled_data_->index_.x, eval_vtx->bundled_data_->index_.y, 0);
	Eigen::Vector3d dir_vec = max_rwd_vec - pos_vec;
	// angle = atan2(dir_vec(1), dir_vec(0));
	angle = -atan2(dir_vec(0), dir_vec(1));
	// std::cout << "id: " << eval_vtx->bundled_data_->data_id_ <<" ;dir vec: " << dir_vec(0) << " , " << dir_vec(1) << " ; angle: " << angle/M_PI*180.0 << std::endl;

	// Eigen::Vector3d x_vec(0,-1,0);
	// Eigen::Vector3d y_vec(-1,0,0);
	// double x_dir_vec = dir_vec.dot(x_vec);
	// double y_dir_vec = dir_vec.dot(y_vec);

	// if(y_dir_vec > 0) {
	// 	angle = std::acos(dir_vec.normalized().dot(x_vec));
	// }
	// else if(y_dir_vec < 0) {
	// 	angle = - std::acos(dir_vec.normalized().dot(x_vec));
	// }
	// else {
	// 	if(x_dir_vec >= 0)
	// 		angle = 0;
	// 	else
	// 		angle = M_PI;
	// }

	eval_vtx->rewards_yaw_ = angle/M_PI*180.0;

	return max_rwd;
}

void ShortcutEval::EvaluateGridShortcutPotential(uint16_t sensor_range)
{
	std::priority_queue<double> all_rewards;
	auto vertices = nav_field_->field_graph_->GetGraphVertices();

	for(auto& vtx : vertices) {
		if(vtx->potential_ == std::numeric_limits<double>::infinity())
			continue;
		vtx->shortcut_rewards_ = EvaluateCellShortcutPotential(vtx, sensor_range);
		all_rewards.push(vtx->shortcut_rewards_);
	}

	nav_field_->max_rewards_ = all_rewards.top();

	std::cout << "max possible rewards: " << nav_field_->max_rewards_ << std::endl;
}

Path_t<SquareCell*> ShortcutEval::SearchInNavField(Vertex_t<SquareCell*>* start_vtx, Vertex_t<SquareCell*>* goal_vtx)
{
	nav_field_->field_graph_->ResetGraphVertices();

	bool found_path = false;
	Path_t<SquareCell*> path;
	Vertex_t<SquareCell*>* current_vertex;

	// open list - a list of vertices that need to be checked out
	PriorityQueue<Vertex_t<SquareCell*>*> openlist;

	openlist.put(start_vtx, 0);
	start_vtx->is_in_openlist_ = true;

	start_vtx->g_astar_ = 0;
	start_vtx->shortcut_rewards_ = 0;
	start_vtx->reward_num_ = 0;
	start_vtx->weighted_cost_ = nav_field_->max_rewards_;// 0;

	while(!openlist.empty() && found_path != true)
	{
		current_vertex = openlist.get();
		if(current_vertex->is_checked_)
			continue;

		current_vertex->is_in_openlist_ = false;
		current_vertex->is_checked_ = true;

		// check all adjacent vertices (successors of current vertex)
		for(auto ite = current_vertex->edges_.begin(); ite != current_vertex->edges_.end(); ite++)
		{
			Vertex_t<SquareCell*>* successor;
			successor = (*ite).dst_;

			// check if the vertex has been checked (in closed list)
			if(successor->is_checked_ == false)
			{
				// first set the parent of the adjacent vertex to be the current vertex
//				double avg_rewards = 0;
//				if(successor->shortcut_rewards_ != 0)
//					avg_rewards = nav_field_->max_rewards_ - (successor->shortcut_rewards_ + current_vertex->shortcut_avg_*current_vertex->reward_num_)/(current_vertex->reward_num_ + 1);
//				else
//					avg_rewards = nav_field_->max_rewards_ - current_vertex->shortcut_avg_;
				//double avg_rewards = nav_field_->max_rewards_ - std::max(current_vertex->shortcut_rewards_, successor->shortcut_rewards_);
//				double avg_rewards = nav_field_->max_rewards_ - (successor->shortcut_rewards_ + current_vertex->shortcut_avg_*current_vertex->reward_num_)/(current_vertex->reward_num_ + 1);
				// no penalty for leaving shortcut regions
				// rewards for entering shortcut regions
				// rewards for staying in high-reward cells
//				double region_rewards = current_vertex->shortcut_rewards_ - successor->shortcut_rewards_;
//				if(region_rewards > 0)
//					region_rewards = 0;
				double new_rewards = current_vertex->shortcut_cost_ + (current_vertex->shortcut_rewards_ - successor->shortcut_rewards_ ) + (1-successor->shortcut_rewards_/nav_field_->max_rewards_)*sgrid_->cell_size_;
				// equivalent cost function
				//double new_rewards = current_vertex->shortcut_cost_ + (1-successor->shortcut_rewards_/nav_field_->max_rewards_)*sgrid_->cell_size_;
				double new_dist = current_vertex->g_astar_ + (*ite).cost_;

				//double new_cost = current_vertex->weighted_cost_ + (new_dist*dist_weight + new_rewards*(1-dist_weight));
				double new_cost = new_dist*dist_weight_ + new_rewards*(1-dist_weight_);// + avg_rewards; // avg_rewards*(1-dist_weight);

				// if the vertex is not in open list
				// or if the vertex is in open list but has a higher cost
				if(successor->is_in_openlist_ == false || new_cost < successor->weighted_cost_)
				{
					successor->search_parent_ = current_vertex;
					successor->weighted_cost_ = new_cost;

//					if(successor->shortcut_rewards_ != 0)
//					{
//						successor->shortcut_avg_ = avg_rewards;
//						successor->reward_num_ = current_vertex->reward_num_ + 1;
//					}
//					else
//					{
//						successor->shortcut_avg_ = current_vertex->shortcut_avg_;
//						successor->reward_num_ = current_vertex->reward_num_;
//					}
					successor->shortcut_cost_ = new_rewards;
					successor->g_astar_ = new_dist;

					openlist.put(successor, successor->weighted_cost_);
					successor->is_in_openlist_ = true;

					if(successor == goal_vtx){
						found_path = true;
					}
				}
			}
		}
	}

	if(found_path)
	{
		std::cout << "path found in nav field" << std::endl;
		Vertex_t<SquareCell*>* waypoint = goal_vtx;
		while(waypoint != start_vtx)
		{
			path.push_back(waypoint);
			waypoint = waypoint->search_parent_;
		}
		// add the start node
		path.push_back(waypoint);
		std::reverse(path.begin(), path.end());

		auto traj_s = path.begin();
		auto traj_e = path.end() - 1;
#ifndef MINIMAL_PRINTOUT
		std::cout << "starting vertex id: " << (*traj_s)->vertex_id_ << std::endl;
		std::cout << "finishing vertex id: " << (*traj_e)->vertex_id_ << std::endl;
		std::cout << "path length: " << path.size() << std::endl;
		std::cout << "total dist cost: " << path.back()->g_astar_ << std::endl;
		std::cout << "total shortcut cost: " << path.back()->shortcut_cost_ << std::endl;
		std::cout << "total cost with rewards: " << path.back()->weighted_cost_ << std::endl;
#endif
	}
	else
		std::cout << "failed to find a path in nav field" << std::endl;

	return path;
}
