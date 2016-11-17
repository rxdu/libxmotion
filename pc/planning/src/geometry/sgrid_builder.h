/*
 * sgrid_builder.h
 *
 *  Created on: Mar 23, 2016
 *      Author: rdu
 */

#ifndef SRC_MAP_SGRID_BUILDER_H_
#define SRC_MAP_SGRID_BUILDER_H_

#include <vector>
#include <tuple>
#include <cstdint>
#include <memory>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "square_grid/square_grid.h"
#include "map/map_type.h"

namespace srcl_ctrl {

class SGridBuilder
{
public:
	SGridBuilder();
	~SGridBuilder();

public:
	static std::shared_ptr<SquareGrid> BuildSquareGrid(cv::InputArray _src, uint32_t cell_size);
	static std::shared_ptr<SquareGrid> BuildExtSquareGrid(cv::InputArray _src, uint32_t cell_size, uint8_t expand_num);

	static Map_t<SquareGrid> BuildSquareGridMap(cv::InputArray _src, uint32_t cell_size);
	static Map_t<SquareGrid> BuildSquareGridMapWithExtObstacle(cv::InputArray _src, uint32_t cell_size, uint8_t expand_num);
};

}

#endif /* SRC_MAP_SGRID_BUILDER_H_ */
