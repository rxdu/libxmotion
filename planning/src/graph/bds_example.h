/*
 * bds_example.h
 *
 *  Created on: Apr 15, 2016
 *      Author: rdu
 */

#ifndef SRC_GRAPH_BDS_EXAMPLE_H_
#define SRC_GRAPH_BDS_EXAMPLE_H_

#include <bds_base.h>

namespace srcl_ctrl {

/****************************************************************************/
/*				   Bundled Data Structure (BDS) Example						*/
/****************************************************************************/
/// An example node that can be associated with a vertex. This node can be
///	either a "struct" or a "class", only need to provide the node_id_ attribute.
struct BDSExample: public BDSBase<BDSExample>
{
	BDSExample(uint64_t id):
		BDSBase<BDSExample>(id){};
	~BDSExample(){};

	double GetHeuristic(const BDSExample& other_struct) const {
		return 1.0;
	}
};

}

#endif /* SRC_GRAPH_BDS_EXAMPLE_H_ */
