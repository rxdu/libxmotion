/*
 * map_info.h
 *
 *  Created on: Jul 27, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_MAP_DATA_MAP_INFO_H_
#define PLANNING_SRC_MAP_DATA_MAP_INFO_H_

namespace srcl_ctrl {

// Origin is defined at the top left corner.
//	X-axis increases from left to right.
//	Y-axis increases from top to bottom.
typedef struct {
	bool vector_map;
	double world_size_x;	// in meters
	double world_size_y;
	uint32_t map_size_x;	// in pixels
	uint32_t map_size_y;
	uint32_t padded_size_x;	// in pixels
	uint32_t padded_size_y;
	double scale;
} MapInfo;

}


#endif /* PLANNING_SRC_MAP_DATA_MAP_INFO_H_ */
