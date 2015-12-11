/*
 * qtree_types.h
 *
 *  Created on: Dec 10, 2015
 *      Author: rdu
 */

#ifndef SRC_QUADTREE_QTREE_TYPES_H_
#define SRC_QUADTREE_QTREE_TYPES_H_

namespace srcl_ctrl {

// Definition of Supporting Types
enum class NodeType
{
	INNER,
	LEAF,
	DUMMY
};

enum class OccupancyType
{
	FREE,
	OCCUPIED,
	MIXED
};

typedef struct
{
	uint32_t min;
	uint32_t max;
}NodeRange;

typedef struct
{
	NodeRange x;
	NodeRange y;
}BoundingBox;

}

#endif /* SRC_QUADTREE_QTREE_TYPES_H_ */
