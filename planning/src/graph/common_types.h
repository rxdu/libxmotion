/*
 * common_types.h
 *
 *  Created on: Jan 22, 2016
 *      Author: rdu
 */

#ifndef SRC_GRAPH_COMMON_TYPES_H_
#define SRC_GRAPH_COMMON_TYPES_H_

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

}

#endif /* SRC_GRAPH_COMMON_TYPES_H_ */
