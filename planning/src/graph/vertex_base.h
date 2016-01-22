/*
 * vertex_base.h
 *
 *  Created on: Jan 22, 2016
 *      Author: rdu
 */

#ifndef SRC_GRAPH_VERTEX_BASE_H_
#define SRC_GRAPH_VERTEX_BASE_H_

#include "common_types.h"

namespace srcl_ctrl{

class VertexBase
{
public:
	VertexBase():
		vertex_id_(0){};
	~VertexBase(){};

public:
	uint64_t vertex_id_;
};

}

#endif /* SRC_GRAPH_VERTEX_BASE_H_ */
