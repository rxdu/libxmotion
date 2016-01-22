/*
 * vertex_base.h
 *
 *  Created on: Jan 22, 2016
 *      Author: rdu
 */

#ifndef SRC_GRAPH_VERTEX_BASE_H_
#define SRC_GRAPH_VERTEX_BASE_H_

namespace srcl_ctrl{

typedef struct
{
	uint32_t x;
	uint32_t y;
}Position2D;

typedef struct
{
	uint32_t x;
	uint32_t y;
	uint32_t z;
}Position3D;

class VertexBase
{
public:
	VertexBase():
		vertex_id_(0){};
	~VertexBase(){};

public:
	Position2D vertex_pos_;
	uint64_t vertex_id_;
};

}

#endif /* SRC_GRAPH_VERTEX_BASE_H_ */
