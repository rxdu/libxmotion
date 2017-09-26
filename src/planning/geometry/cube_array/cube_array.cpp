/*
 * cube_array.cpp
 *
 *  Created on: Sep 8, 2016
 *      Author: rdu
 */

#include <iostream>

#include "cube_array/cube_array.h"

using namespace librav;

CubeArray::CubeArray(int32_t col_num, int32_t row_num, int32_t height_num, double cube_size):
				row_size_(row_num),
				col_size_(col_num),
				hei_size_(height_num),
				row_offset_(0),
				col_offset_(0),
				hei_offset_(0),
				cube_size_(cube_size)
{
	cubes_.resize(row_size_*col_size_*hei_size_);

	for(uint32_t hei = 0; hei < hei_size_; hei++)
		for(uint32_t row = 0; row < row_size_; row++)
			for(uint32_t col = 0; col < col_size_; col++)
			{
				uint64_t id = GetIDFromIndex(col, row, hei);
				cubes_[id].data_id_ = id;
				cubes_[id].index_.x = col;
				cubes_[id].index_.y = row;
				cubes_[id].index_.z = hei;
				cubes_[id].location_.x = cube_size_/2.0 + static_cast<int32_t>(col) * cube_size_;
				cubes_[id].location_.y = cube_size_/2.0 + static_cast<int32_t>(row) * cube_size_;
				cubes_[id].location_.z = cube_size_/2.0 + static_cast<int32_t>(hei) * cube_size_;
				cubes_[id].occu_ = OccupancyType::UNKONWN;

				//				std::cout << "id: " << id <<
				//						", coordinate: "
				//						<< cubes_[id].location_.x << " , "
				//						<< cubes_[id].location_.y << " , "
				//						<< cubes_[id].location_.z<<
				//						", index: "
				//						<< cubes_[id].index_.x << " , "
				//						<< cubes_[id].index_.y << " , "
				//						<< cubes_[id].index_.z << std::endl;
			}
}

void CubeArray::SetOriginOffset(int32_t col_offset, int32_t row_offset, int32_t hei_offset)
{
	row_offset_ = row_offset;
	col_offset_ = col_offset;
	hei_offset_ = hei_offset;

	for(uint32_t hei = 0; hei < hei_size_; hei++)
		for(uint32_t row = 0; row < col_size_; row++)
			for(uint32_t col = 0; col < col_size_; col++)
			{
				uint64_t id = GetIDFromIndex(col, row, hei);
				cubes_[id].location_.x = cube_size_/2.0 + (static_cast<int32_t>(col) - col_offset_) * cube_size_;
				cubes_[id].location_.y = cube_size_/2.0 + (static_cast<int32_t>(row) - row_offset_) * cube_size_;
				cubes_[id].location_.z = cube_size_/2.0 + (static_cast<int32_t>(hei) - hei_offset_) * cube_size_;

				//				std::cout << "id: " << id <<
				//						", coordinate: "
				//						<< cubes_[id].location_.x << " , "
				//						<< cubes_[id].location_.y << " , "
				//						<< cubes_[id].location_.z<<
				//						", index: "
				//						<< cubes_[id].index_.x << " , "
				//						<< cubes_[id].index_.y << " , "
				//						<< cubes_[id].index_.z << std::endl;
			}
}

uint64_t CubeArray::GetIDFromIndex(uint32_t col, uint32_t row, uint32_t hei)
{
	uint64_t id = 0;

	id = hei * (row_size_*col_size_) + row*col_size_ + col;

	return id;
}

uint64_t CubeArray::GetIDFromPosition(double x, double y, double z)
{
	uint32_t row, col, hei;

	if(x < -col_offset_ * cube_size_)
		x = -col_offset_ * cube_size_;
	if(y < - row_offset_ * cube_size_)
		y = -row_offset_ * cube_size_;
	if(z < -hei_offset_ * cube_size_)
		z = -hei_offset_ * cube_size_;

	// avoid the top right corner
	if(x > (static_cast<int32_t>(col_size_)-col_offset_) * cube_size_)
		x = (static_cast<int32_t>(col_size_)-col_offset_) * cube_size_ - cube_size_/10.0;
	if(y > (static_cast<int32_t>(row_size_)-row_offset_) * cube_size_)
		y = (static_cast<int32_t>(row_size_)-row_offset_) * cube_size_ - cube_size_/10.0;
	if(z > (static_cast<int32_t>(hei_size_)-hei_offset_) * cube_size_)
		z = (static_cast<int32_t>(hei_size_)-hei_offset_) * cube_size_ - cube_size_/10.0;

	//	std::cout << "(x, y, z): " << x << " , " << y << " , " << z << std::endl;
	//	std::cout << "divide: " << x/cube_size_ << std::endl;
	col = static_cast<int32_t>((x + row_offset_*cube_size_)/cube_size_);
	row = static_cast<int32_t>((y + col_offset_*cube_size_)/cube_size_);
	hei = static_cast<int32_t>((z + hei_offset_*cube_size_)/cube_size_);

	//	std::cout << "(row, col, hei): " << row << " , " << col << " , " << hei << std::endl;
	return GetIDFromIndex(col, row, hei);
}

bool CubeArray::GetCubeHeightIndexAtHeight(double z, std::vector<uint32_t>& hei_set)
{
	int32_t hei;

	if(z < -hei_offset_ * cube_size_)
		z = -hei_offset_ * cube_size_;

	// avoid the top right corner
	if(z > (static_cast<int32_t>(hei_size_)-hei_offset_) * cube_size_)
		z = (static_cast<int32_t>(hei_size_)-hei_offset_) * cube_size_ - cube_size_/10.0;

	hei = static_cast<int32_t>((z + hei_offset_*cube_size_)/cube_size_);

	if(hei >=0 && hei < hei_size_)
		hei_set.push_back(hei);
	else
		return false;

	//	std::cout << "hei: " << hei << std::endl;
	//	std::cout << "current height: " << z << " , nearest cube height: " << (hei + 1) * cube_size_ - hei_offset_*cube_size_ << std::endl;

	if(z - ((hei + 1) * cube_size_ - hei_offset_*cube_size_) > 0)
	{
		if((hei + 1) < hei_size_)
			hei_set.push_back(hei + 1);
	}
	else
	{
		if(hei - 1 >=0)
			hei_set.push_back(hei - 1);
	}

	return true;
}

std::vector<uint64_t> CubeArray::GetFreeCubesAroundHeight(double height)
{
	std::vector<uint64_t> set;

	for(int32_t i = 0; i < col_size_; i++)
		for(int32_t j = 0; j < row_size_; j++) {
			// find node right above or equal to given height
			for(int32_t k = 0; k < hei_size_; k++) {
				uint64_t id = GetIDFromIndex(i, j, k);
				if(cubes_[id].occu_ == OccupancyType::OCCUPIED)
					continue;

				// height at given index
				double ch = (k - hei_offset_) * cube_size_ - cube_size_/2;

				if(ch >= height) {
					set.push_back(id);
					break;
				}
			}
			// find node right below given height
			for(int32_t k = 0; k < hei_size_; k++) {
				uint64_t id = GetIDFromIndex(i, j, k);
				if(cubes_[id].occu_ == OccupancyType::OCCUPIED)
					continue;

				// height at given index
				double ch = (k - hei_offset_) * cube_size_ - cube_size_/2;

				if(ch < height) {
					set.push_back(id);
					break;
				}
			}
		}

	return set;
}

void CubeArray::SetCubeOccupancy(uint32_t col, uint32_t row, uint32_t hei, OccupancyType oc_type)
{
	auto id = GetIDFromIndex(col, row, hei);
	cubes_[id].occu_ = oc_type;
}

void CubeArray::UpdateCubeOccupancy(double x, double y, double z, OccupancyType oc_type)
{
	uint64_t id = GetIDFromPosition(x,y,z);

	cubes_[id].occu_ = oc_type;
}

std::vector<uint64_t> CubeArray::GetNeighbours(uint64_t id)
{
	std::vector<uint64_t> neighbours;

	int64_t row, col, hei;
	col = cubes_[id].index_.x;
	row = cubes_[id].index_.y;
	hei = cubes_[id].index_.z;

	// do not allow diagonal movement
	if(row - 1 >= 0)
		neighbours.push_back(GetIDFromIndex(col, row - 1, hei));
	if(row + 1 < row_size_)
		neighbours.push_back(GetIDFromIndex(col, row + 1, hei));
	if(col - 1 >= 0)
		neighbours.push_back(GetIDFromIndex(col - 1, row, hei));
	if(col + 1 < col_size_)
		neighbours.push_back(GetIDFromIndex(col + 1, row, hei));
	if(hei - 1 >= 0)
		neighbours.push_back(GetIDFromIndex(col, row, hei - 1));
	if(hei + 1 < hei_size_)
		neighbours.push_back(GetIDFromIndex(col, row, hei + 1));

	return neighbours;
}

std::vector<uint64_t> CubeArray::GetNeighbours(uint64_t id, bool allow_diag)
{
	std::vector<uint64_t> neighbours;

	int64_t row, col, hei;
	col = cubes_[id].index_.x;
	row = cubes_[id].index_.y;
	hei = cubes_[id].index_.z;

	// do not allow diagonal movement
	if(!allow_diag)
	{
		if(row - 1 >= 0)
			neighbours.push_back(GetIDFromIndex(col, row - 1, hei));
		if(row + 1 < row_size_)
			neighbours.push_back(GetIDFromIndex(col, row + 1, hei));
		if(col - 1 >= 0)
			neighbours.push_back(GetIDFromIndex(col - 1, row, hei));
		if(col + 1 < col_size_)
			neighbours.push_back(GetIDFromIndex(col + 1, row, hei));
		if(hei - 1 >= 0)
			neighbours.push_back(GetIDFromIndex(col, row, hei - 1));
		if(hei + 1 < hei_size_)
			neighbours.push_back(GetIDFromIndex(col, row, hei + 1));
	}
	else
	{
		for(int i = col - 1; i <= col + 1; i++ )
			for(int j = row - 1; j <= row + 1; j++ )
				for(int k = hei - 1; k <= hei + 1; k++ )
				{
					if(i >= 0 && i < col_size_ &&
							j >= 0 && j < row_size_ &&
							k >= 0 && k < hei_size_)
					{
						if(!(i == col && j == row && k == hei))
							neighbours.push_back(GetIDFromIndex(i, j, k));
					}
				}
	}

	return neighbours;
}

// this function can only be used when octree is built in vehicle frame
std::vector<uint64_t> CubeArray::GetStartingCubes()
{
	std::vector<uint64_t> start_cubes;

	uint64_t test_id;
	test_id = GetIDFromPosition(cube_size_/2, cube_size_/2, cube_size_/2);
	if(isIDValid(test_id) && cubes_[test_id].occu_ == OccupancyType::FREE)
		start_cubes.push_back(test_id);
	test_id = GetIDFromPosition(cube_size_/2, -cube_size_/2, cube_size_/2);
	if(isIDValid(test_id) && cubes_[test_id].occu_ == OccupancyType::FREE)
		start_cubes.push_back(test_id);
	test_id = GetIDFromPosition(cube_size_/2, cube_size_/2, -cube_size_/2);
	if(isIDValid(test_id) && cubes_[test_id].occu_ == OccupancyType::FREE)
		start_cubes.push_back(test_id);
	test_id = GetIDFromPosition(cube_size_/2, -cube_size_/2, -cube_size_/2);
	if(isIDValid(test_id) && cubes_[test_id].occu_ == OccupancyType::FREE)
		start_cubes.push_back(test_id);

	return start_cubes;
}

bool CubeArray::GetCubeIDAtPosition(double x, double y, double z, uint64_t& id)
{
	id = this->GetIDFromPosition(x,y,z);

	if(this->isIDValid(id) && this->cubes_[id].occu_ == OccupancyType::FREE)
		return true;
	else
		return false;

}
