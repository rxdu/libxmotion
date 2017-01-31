/*
 * =====================================================================================
 *
 *       Filename:  nav_field.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  01/17/2017 02:50:12 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ruixiang Du (rdu), ruixiang.du@gmail.com
 *   Organization:  Worcester Polytechnic Institute
 *
 * =====================================================================================
 */

#ifndef SRC_NAV_FIELD_H_
#define SRC_NAV_FIELD_H_

#include <cstdint>
#include <memory>

#include "graph/graph.h"
#include "graph/priority_queue.h"

namespace srcl_ctrl {
	template<class GraphNodeType>
	class NavField {
		public:
			NavField()= delete;
			NavField(std::shared_ptr<Graph_t<GraphNodeType>> graph):
				field_graph_(graph),
				field_center_(nullptr){};
			~NavField(){};

			std::shared_ptr<Graph_t<GraphNodeType>> field_graph_;
			Vertex_t<GraphNodeType>* field_center_;

		private:
			void ConstructNavField(Vertex_t<GraphNodeType>* goal_vtx) {
				field_center_ = goal_vtx;
				field_graph_->ResetGraphVertices();

				bool found_path = false;
				Vertex_t<GraphNodeType>* current_vertex;

				// open list - a list of vertices that need to be checked out
				PriorityQueue<Vertex_t<GraphNodeType>*> openlist;

				openlist.put(goal_vtx, 0);
				goal_vtx->is_in_openlist_ = true;

				//start->search_parent_ = start;
				goal_vtx->potential_ = 0;

				while(!openlist.empty())
				{
					current_vertex = openlist.get();
					if(current_vertex->is_checked_)
						continue;

					current_vertex->is_in_openlist_ = false;
					current_vertex->is_checked_ = true;

					// check all adjacent vertices (successors of current vertex)
					for(auto ite = current_vertex->edges_.begin(); ite != current_vertex->edges_.end(); ite++)
					{
						Vertex_t<GraphNodeType>* successor;
						successor = (*ite).dst_;

						// check if the vertex has been checked (in closed list)
						if(successor->is_checked_ == false)
						{
							// first set the parent of the adjacent vertex to be the current vertex
							double new_cost = current_vertex->potential_ + (*ite).cost_;

							// if the vertex is not in open list
							// or if the vertex is in open list but has a higher cost
							if(successor->is_in_openlist_ == false || new_cost < successor->potential_)
							{
								successor->potential_parent_ = current_vertex;
								successor->potential_ = new_cost;

								openlist.put(successor, successor->potential_);
								successor->is_in_openlist_ = true;
							}
						}
					}
				}

				std::cout << "navigation field updated!" << std::endl;
			};

		public:
			void UpdateNavField(uint64_t goal_id) {
				auto goal_vtx = field_graph_->GetVertexFromID(goal_id);

				ConstructNavField(goal_vtx);	
			};

			Path_t<GraphNodeType> SearchInNavField(Vertex_t<GraphNodeType>* start_vtx, Vertex_t<GraphNodeType>* goal_vtx) {
				ConstructNavField(goal_vtx);
				field_graph_->ResetGraphVertices();

				bool found_path = false;
				Path_t<GraphNodeType> path;
				Vertex_t<GraphNodeType>* current_vertex;
				// open list - a list of vertices that need to be checked out
				PriorityQueue<Vertex_t<GraphNodeType>*> openlist;

				openlist.put(start_vtx, 0);
				start_vtx->is_in_openlist_ = true;

				//start->search_parent_ = start;
				start_vtx->g_astar_ = 0;

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
						Vertex_t<GraphNodeType>* successor;
						successor = (*ite).dst_;

						// check if the vertex has been checked (in closed list)
						if(successor->is_checked_ == false)
						{
							// first set the parent of the adjacent vertex to be the current vertex
							double new_cost = current_vertex->g_astar_ + (*ite).cost_;

							// if the vertex is not in open list
							// or if the vertex is in open list but has a higher cost
							if(successor->is_in_openlist_ == false || new_cost < successor->g_astar_)
							{
								successor->search_parent_ = current_vertex;
								successor->g_astar_ = new_cost;

								openlist.put(successor, successor->g_astar_);
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
					Vertex_t<GraphNodeType>* waypoint = goal_vtx;
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
#ifdef MINIMAL_PRINTOUT
					std::cout << "starting vertex id: " << (*traj_s)->vertex_id_ << std::endl;
					std::cout << "finishing vertex id: " << (*traj_e)->vertex_id_ << std::endl;
					std::cout << "path length: " << path.size() << std::endl;
					std::cout << "total cost: " << path.back()->g_astar_ << std::endl;
#endif
				}
				else
					std::cout << "failed to find a path in nav field" << std::endl;

				return path;
			}
	};
}

#endif /* SRC_NAV_FIELD_H_ */
