/*
 * bundled_struct_base.h
 *
 *  Created on: Apr 14, 2016
 *      Author: rdu
 */

#ifndef SRC_GRAPH_BUNDLED_STRUCT_BASE_H_
#define SRC_GRAPH_BUNDLED_STRUCT_BASE_H_

namespace srcl_ctrl {

class BundledStructBase {
public:
	BundledStructBase();
	virtual ~BundledStructBase();

public:
	virtual double CalcHeuristic(BundledStructBase* other_struct) = 0;
};

}

#endif /* SRC_GRAPH_BUNDLED_STRUCT_BASE_H_ */
