/*
 * bds_base.h
 *
 *  Created on: Apr 14, 2016
 *      Author: rdu
 */

#ifndef SRC_GRAPH_BDS_BASE_H_
#define SRC_GRAPH_BDS_BASE_H_

#include <cstdint>

namespace srcl_ctrl {

/****************************************************************************/
/*				   Bundled Data Structure (BDS) Base						*/
/****************************************************************************/

template<typename BundledDataStructType>
class BDSBase {
public:
	BDSBase():data_id_(0){};
	BDSBase(uint64_t struct_id):data_id_(struct_id){};
	virtual ~BDSBase(){};

public:
	uint64_t data_id_;

public:
	virtual double GetHeuristic(const BundledDataStructType& other_struct) const  = 0;
};

}

#endif /* SRC_GRAPH_BDS_BASE_H_ */
