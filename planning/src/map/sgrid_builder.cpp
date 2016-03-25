/*
 * sgrid_builder.cpp
 *
 *  Created on: Mar 23, 2016
 *      Author: rdu
 */

#include "sgrid_builder.h"
#include "image_utils.h"

using namespace srcl_ctrl;
using namespace cv;

SGridBuilder::SGridBuilder()
{

}

SGridBuilder::~SGridBuilder()
{

}

SquareGrid* SGridBuilder::BuildSquareGrid(cv::InputArray _src, uint32_t cell_size)
{
	Mat image_bin;
	Mat image_map;
	Mat src = _src.getMat();

	// Binarize grayscale image
	ImageUtils::BinarizeImage(src, image_bin, 200);

	// pad image to 2^n on each side so that we can calculate
	//	the dimension of the grid more conveniently
	ImageUtils::PadImageTo2Exp(image_bin, image_map);

	// Create quadtree
	uint32_t row_num = image_map.rows / cell_size;
	uint32_t col_num = image_map.cols / cell_size;
	SquareGrid *sgrid = new SquareGrid(row_num,col_num, cell_size);

	// set occupancy of each cell
	for(uint32_t i = 0; i < row_num; i++)
		for(uint32_t j = 0; j < col_num; j++)
		{
			uint32_t id = sgrid->GetIDFromPosition(i,j);
			if(ImageUtils::CheckAreaOccupancy(image_map, sgrid->cells_[id]->bbox_) == OccupancyType::OCCUPIED ||
					ImageUtils::CheckAreaOccupancy(image_map, sgrid->cells_[id]->bbox_) == OccupancyType::MIXED)
				sgrid->SetCellOccupancy(id, OccupancyType::OCCUPIED);
		}

	return sgrid;
}
