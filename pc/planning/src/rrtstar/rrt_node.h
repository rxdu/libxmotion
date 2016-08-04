/*
 * rrt_node.h
 *
 *  Created on: Aug 4, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_RRTSTAR_RRT_NODE_H_
#define PLANNING_SRC_RRTSTAR_RRT_NODE_H_

#include "graph/bds_base.h"
#include "common/planning_types.h"

namespace srcl_ctrl {

/****************************************************************************/
/*				   Bundled Data Structure (BDS) Example						*/
/****************************************************************************/
/// An example BDS that can be associated with a vertex. This BDS can be
///	either a "struct" or a "class", but need to provide an implementation of
/// the GetHeuristic() function. A "data_id_" is provided with the base class.
struct RRTNode: public BDSBase<RRTNode>
{
	RRTNode(uint64_t id):
		BDSBase<RRTNode>(id){};
	~RRTNode(){};

	Position2Dd position;

	double GetHeuristic(const RRTNode& other_struct) const {
		return 0.0;
	}
};

}

#endif /* PLANNING_SRC_RRTSTAR_RRT_NODE_H_ */
