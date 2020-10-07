/* 
 * sgrid_builder.cpp
 * 
 * Created on: Mar 23, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */ 

#include "map/sgrid_builder.hpp"

using namespace autodrive;
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

std::shared_ptr<SquareGrid> SGridBuilderV2::BuildExtSquareGrid(std::shared_ptr<SquareGrid> original_grid, uint8_t obj_expand_num)
{	
	std::shared_ptr<SquareGrid> sgrid = std::make_shared<SquareGrid>(original_grid->col_size_,original_grid->row_size_, original_grid->cell_size_);

	uint32_t row_num = original_grid->row_size_;
	uint32_t col_num = original_grid->col_size_;
	// set occupancy of each cell
	for(uint32_t i = 0; i < col_num; i++)
		for(uint32_t j = 0; j < row_num; j++)
		{
			uint32_t id = sgrid->GetIDFromIndex(i,j);
			if(original_grid->cells_[id]->occu_ == OccupancyType::OCCUPIED)
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
							sgrid->cells_[checked_id]->occu_ = OccupancyType::OCCUPIED;
					}
				}
		}

		std::cout << "extended obstacles on the map" << std::endl;
	}

	return sgrid;
}
