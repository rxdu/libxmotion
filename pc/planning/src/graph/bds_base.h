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
/// The base class of bundled data structure
template<typename BundledDataStructType>
class BDSBase {
protected:
	// this enforces the derived class to initialize the id somehow, just to make
	//	sure "data_id_" is used as the ID, not with any other names
	BDSBase() = delete;
	BDSBase(uint64_t struct_id):data_id_(struct_id){};
	virtual ~BDSBase(){};

public:
	uint64_t data_id_;

public:
	uint64_t GetID() const {return data_id_;}
	virtual double GetHeuristic(const BundledDataStructType& other_struct) const  = 0;
};

}

#endif /* SRC_GRAPH_BDS_BASE_H_ */
