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

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "square_grid/square_grid.h"

namespace srcl_ctrl {

class SGridBuilder
{
public:
	SGridBuilder();
	~SGridBuilder();

public:
	static SquareGrid* BuildSquareGrid(cv::InputArray _src, uint32_t cell_size);
	static std::tuple<SquareGrid*, cv::Mat> BuildSquareGridMap(cv::InputArray _src, uint32_t cell_size);
};

}

#endif /* SRC_MAP_SGRID_BUILDER_H_ */
