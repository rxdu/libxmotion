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

#include "planning/graph/graph.h"
#include "planning/graph/priority_queue.h"

namespace librav {
	template<class GraphNodeType>
	class NavField {
		public:
			NavField()= delete;
			NavField(std::shared_ptr<Graph_t<GraphNodeType>> graph):
				field_graph_(graph),
				field_center_(nullptr),
				max_rewards_(0){};
			~NavField(){};

			std::shared_ptr<Graph_t<GraphNodeType>> field_graph_;
			Vertex_t<GraphNodeType>* field_center_;
			double max_rewards_;

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
	};
}

#endif /* SRC_NAV_FIELD_H_ */
