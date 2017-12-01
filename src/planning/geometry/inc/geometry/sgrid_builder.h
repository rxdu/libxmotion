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

#include "geometry/square_grid.h"
#include "map/map_type.h"

namespace librav {

namespace SGridBuilder
{
	std::shared_ptr<SquareGrid> BuildSquareGrid(cv::InputArray _src, uint32_t cell_size);
	std::shared_ptr<SquareGrid> BuildExtSquareGrid(cv::InputArray _src, uint32_t cell_size, uint8_t expand_num);

	Map_t<SquareGrid> BuildSquareGridMap(cv::InputArray _src, uint32_t cell_size);
	Map_t<SquareGrid> BuildSquareGridMapWithExtObstacle(cv::InputArray _src, uint32_t cell_size, uint8_t expand_num);
};

namespace SGridBuilderV2
{
	std::shared_ptr<SquareGrid> BuildSquareGrid(cv::InputArray _src, uint32_t cell_size, uint8_t obj_expand_num = 0, int64_t img_offset_x = 0, int64_t img_offset_y = 0);
	std::shared_ptr<SquareGrid> BuildExtSquareGrid(cv::InputArray _src, uint32_t cell_size, uint8_t expand_num);
	std::shared_ptr<SquareGrid> BuildExtSquareGrid(std::shared_ptr<SquareGrid> original_grid, uint8_t expand_num);

	Map_t<SquareGrid> BuildSquareGridMap(cv::InputArray _src, uint32_t cell_size, uint8_t obj_expand_num = 0);
	Map_t<SquareGrid> BuildSquareGridMapWithExtObstacle(cv::InputArray _src, uint32_t cell_size, uint8_t expand_num);
};

}

#endif /* SRC_MAP_SGRID_BUILDER_H_ */
