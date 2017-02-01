/*
 * shortcut_eval.cpp
 *
 *  Created on: Jan 31, 2017
 *      Author: rdu
 */

#include <cmath>
#include <queue>

#include "graph/priority_queue.h"
#include "nav_field/shortcut_eval.h"

using namespace srcl_ctrl;

ShortcutEval::ShortcutEval(std::shared_ptr<SquareGrid> sgrid, std::shared_ptr<NavField<SquareCell*>> nav_field):
			sgrid_(sgrid),
			nav_field_(nav_field),
			dist_weight(1.0)
{

}

double ShortcutEval::CalcDirectDistance(Position2D start, Position2D goal, double cell_size, bool allow_diag)
{
	double dist = 0;

	uint32_t x_error, y_error;

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
		uint32_t diag_steps = 0;
		uint32_t straight_steps = 0;

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

double ShortcutEval::EvaluateCellShortcutPotential(Vertex_t<SquareCell*>* eval_vtx)
{
	auto nbs = sgrid_->GetNeighboursWithinRange(eval_vtx->bundled_data_->data_id_, 5);

	std::priority_queue<double> rewards_queue;
	for(auto& n : nbs) {
		if(n->occu_ == OccupancyType::OCCUPIED)
			continue;

		auto vtx = nav_field_->field_graph_->GetVertexFromID(n->data_id_);

		rewards_queue.push(eval_vtx->potential_ - vtx->potential_
				- CalcDirectDistance(vtx->bundled_data_->index_, eval_vtx->bundled_data_->index_,sgrid_->cell_size_,true));
	}

	//eval_vtx->shortcut_rewards_ = rewards_queue.top();
	return rewards_queue.top();
}

void ShortcutEval::EvaluateGridShortcutPotential()
{
	std::priority_queue<double> all_rewards;
	auto vertices = nav_field_->field_graph_->GetGraphVertices();

	for(auto& vtx : vertices) {
		vtx->shortcut_rewards_ = EvaluateCellShortcutPotential(vtx);
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

	//start->search_parent_ = start;
	start_vtx->g_astar_ = 0;
	start_vtx->g_rewards_ = 0;

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
				double new_cost = current_vertex->g_rewards_ + ((*ite).cost_*dist_weight + (nav_field_->max_rewards_ - successor->shortcut_rewards_)*0.5*(1-dist_weight));
				double new_dist = current_vertex->g_astar_ + (*ite).cost_;

				// if the vertex is not in open list
				// or if the vertex is in open list but has a higher cost
				if(successor->is_in_openlist_ == false || new_cost < successor->g_rewards_)
				{
					successor->search_parent_ = current_vertex;
					successor->g_rewards_ = new_cost;
					successor->g_astar_ = new_dist;

					openlist.put(successor, successor->g_rewards_);
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
		std::cout << "total cost with rewards: " << path.back()->g_rewards_ << std::endl;
#endif
	}
	else
		std::cout << "failed to find a path in nav field" << std::endl;

	return path;
}

