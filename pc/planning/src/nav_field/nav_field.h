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

#include <cstdint>
#include <memory>

#include <graph/graph.h>

namespace srcl_ctrl {
	template<class GraphNodeType>
	class NavField {
		public:
			NavField() {};
			NavField(std::shared_ptr<Graph_t<GraphNodeType>> graph){
				nav_field_ = std::make_shared<Graph_t<GraphNodeType>>(*graph);
			};
			~NavField(){};

		private:
			std::shared_ptr<Graph_t<GraphNodeType>> nav_field_;
			void ConstructNavField(Vertex_t<GraphNodeType> goal_vtx) {

			};

		public:
			void UpdateNavField(uint64_t goal_id) {
				auto goal_vtx = nav_field_->GetVertexFromID(goal_id);


			};
	};
}
