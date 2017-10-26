/*
 * sgrid_builder.cpp
 *
 *  Created on: Mar 23, 2016
 *      Author: rdu
 */

#include "map/image_utils.h"
#include "geometry/sgrid_builder.h"

using namespace librav;
using namespace cv;

std::shared_ptr<SquareGrid> SGridBuilderV2::BuildSquareGrid(cv::InputArray _src, uint32_t cell_size, uint8_t obj_expand_num, int64_t img_offset_x, int64_t img_offset_y)
{
	Mat image_map = _src.getMat();

	// calculate square grid params
	uint32_t row_num = image_map.rows / cell_size;
	uint32_t col_num = image_map.cols / cell_size;

	// create square grid
	std::shared_ptr<SquareGrid> sgrid = std::make_shared<SquareGrid>(col_num,row_num, cell_size, img_offset_x, img_offset_y);

	// set occupancy of each cell
	for(uint32_t i = 0; i < col_num; i++)
		for(uint32_t j = 0; j < row_num; j++)
		{
			uint32_t id = sgrid->GetIDFromIndex(i,j);
			OccupancyType type = ImageUtils::CheckAreaOccupancy(image_map, sgrid->cells_[id]->bbox_) ;
			if(type == OccupancyType::OCCUPIED || type == OccupancyType::MIXED)
				sgrid->SetCellOccupancy(id, OccupancyType::OCCUPIED);
		}

	std::vector<uint64_t> edge_obs_cells;
	if(obj_expand_num > 0)
	{
		for(uint32_t i = 0; i < sgrid->col_size_; i++)
			for(uint32_t j = 0; j < sgrid->row_size_; j++)
			{
				uint32_t id = sgrid->GetIDFromIndex(i,j);
				uint8_t free_neighbour_size = 0;

				for(auto& nei : sgrid->GetNeighbours(id, false))
				{
					if(nei->occu_ == OccupancyType::FREE)
						free_neighbour_size++;
				}

				if((sgrid->cells_[id]->occu_ == OccupancyType::OCCUPIED) &&
						(free_neighbour_size > 0 &&  free_neighbour_size < 4))
					edge_obs_cells.push_back(id);
			}

		for(auto& cid : edge_obs_cells)
		{
			for(int i = sgrid->cells_[cid]->index_.x - obj_expand_num; i <= sgrid->cells_[cid]->index_.x + obj_expand_num; i++ )
				for(int j = sgrid->cells_[cid]->index_.y - obj_expand_num; j <= sgrid->cells_[cid]->index_.y + obj_expand_num; j++ )
				{
					if(i >= 0 && i < sgrid->col_size_ &&
							j >= 0 && j < sgrid->row_size_)
					{
						uint32_t checked_id = sgrid->GetIDFromIndex(i,j);

						if(checked_id != cid && sgrid->cells_[checked_id]->occu_ != OccupancyType::OCCUPIED)
							sgrid->cells_[checked_id]->occu_ = OccupancyType::EXPANDED_OBS;
					}
				}
		}

		std::cout << "extended obstacles on the map" << std::endl;
	}

	return sgrid;
}

Map_t<SquareGrid> SGridBuilderV2::BuildSquareGridMap(cv::InputArray _src, uint32_t cell_size, uint8_t obj_expand_num)
{
	Mat src = _src.getMat();

	Map_t<SquareGrid> map;
	map.input_image = src;

	// binarize grayscale image
	ImageUtils::BinarizeImage(src, map.padded_image, 200);

	uint32_t row_num = map.padded_image.rows / cell_size;
	uint32_t col_num = map.padded_image.cols / cell_size;

	int64_t img_offset_x = (map.padded_image.cols - col_num * cell_size)/2;
	int64_t img_offset_y = (map.padded_image.rows - row_num * cell_size)/2;

	// generate map info
	map.info.vector_map = false;
	map.info.map_size_x = map.input_image.cols - 1;
	map.info.map_size_y = map.input_image.rows - 1;
	map.info.padded_top = 0;//img_offset_y;
	map.info.padded_bottom = 0;//img_offset_y;
	map.info.padded_right = 0;//img_offset_x;
	map.info.padded_left = 0;//img_offset_x;

	map.info.scale_x = 1; //static_cast<double>(map.info.map_size_x)/map.info.world_size_x;
	map.info.scale_y = 1; //static_cast<double>(map.info.map_size_y)/map.info.world_size_y;

	// to be updated by specific data structure (square grid/quadtree)
	map.info.resolution = 0;

	// create square grid
	map.data_model = SGridBuilderV2::BuildSquareGrid(map.padded_image, cell_size, obj_expand_num, img_offset_x, img_offset_y);

	return map;
}
