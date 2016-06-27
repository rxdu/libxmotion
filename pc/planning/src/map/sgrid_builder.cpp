/*
 * sgrid_builder.cpp
 *
 *  Created on: Mar 23, 2016
 *      Author: rdu
 */

#include <map/image_utils.h>
#include <map/sgrid_builder.h>

using namespace srcl_ctrl;
using namespace cv;

SGridBuilder::SGridBuilder()
{

}

SGridBuilder::~SGridBuilder()
{

}

std::shared_ptr<SquareGrid> SGridBuilder::BuildSquareGrid(cv::InputArray _src, uint32_t cell_size)
{
	Mat image_bin;
	Mat image_map;
	Mat src = _src.getMat();

	// binarize grayscale image
	ImageUtils::BinarizeImage(src, image_bin, 200);

	// pad image to 2^n on each side so that we can calculate
	//	the dimension of the grid more conveniently
	ImageUtils::PadImageTo2Exp(image_bin, image_map);

	// create square grid
	uint32_t row_num = image_map.rows / cell_size;
	uint32_t col_num = image_map.cols / cell_size;
	std::shared_ptr<SquareGrid> sgrid = std::make_shared<SquareGrid>(row_num,col_num, cell_size);

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

std::tuple<std::shared_ptr<SquareGrid>, cv::Mat> SGridBuilder::BuildSquareGridMap(cv::InputArray _src, uint32_t cell_size)
{
	Mat image_bin;
	Mat image_map;
	Mat src = _src.getMat();

	// binarize grayscale image
	ImageUtils::BinarizeImage(src, image_bin, 200);

	// pad image to 2^n on each side so that we can calculate
	//	the dimension of the grid more conveniently
	ImageUtils::PadImageTo2Exp(image_bin, image_map);

	// create square grid
	std::shared_ptr<SquareGrid> sgrid = SGridBuilder::BuildSquareGrid(_src, cell_size);

	return std::make_tuple(sgrid, image_map);
}
