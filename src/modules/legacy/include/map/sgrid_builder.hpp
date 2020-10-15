/* 
 * sgrid_builder.h
 * 
 * Created on: Mar 23, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef SGRID_BUILDER_H
#define SGRID_BUILDER_H

#include <vector>
#include <tuple>
#include <cstdint>
#include <memory>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "map/square_grid.hpp"

namespace ivnav
{

namespace SGridBuilderV2
{
std::shared_ptr<SquareGrid> BuildSquareGrid(cv::InputArray _src, uint32_t cell_size, uint8_t obj_expand_num = 0, int64_t img_offset_x = 0, int64_t img_offset_y = 0);
std::shared_ptr<SquareGrid> BuildExtSquareGrid(cv::InputArray _src, uint32_t cell_size, uint8_t expand_num);
std::shared_ptr<SquareGrid> BuildExtSquareGrid(std::shared_ptr<SquareGrid> original_grid, uint8_t expand_num);
};
}

#endif /* SGRID_BUILDER_H */
