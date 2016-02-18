/*
 * map_manager.h
 *
 *  Created on: Jan 28, 2016
 *      Author: rdu
 */

#ifndef SRC_MAP_MAP_MANAGER_H_
#define SRC_MAP_MAP_MANAGER_H_

// C++ STL headers
#include <cstdint>

// OpenCV headers
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

// User-defined headers
#include "square_grid.h"

namespace srcl_ctrl{

class MapManager{
public:
	MapManager();
	~MapManager();

private:

public:
	SquareGrid* CreateTestGridMap12N12Case1();
	SquareGrid* CreateTestGridMap12N12Case2();
	SquareGrid* CreateTestGridMap25N25Case1();
	SquareGrid* CreateTestGridMap12N12Astar();
	SquareGrid* CreateTestGridMap3N3();
//	SquareGrid* CreateSquareGridMap(uint32_t row_num, uint32_t col_num, uint32_t square_size);
//	void SetUniCellMapOccupancy(uint32_t row, uint32_t col, OccupancyType occ);

};

}



#endif /* SRC_MAP_MAP_MANAGER_H_ */
