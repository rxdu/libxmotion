/* 
 * cube_array.hpp
 * 
 * Created on: Sep 8, 2016
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CUBE_ARRAY_HPP
#define CUBE_ARRAY_HPP

#include <map>
#include <vector>

#include "adtypes/adtypes.hpp"

namespace ivnav
{

/*
 * Coordinate System:
 *		y
 *	^	^
 *	^	|
 *		|
 * row	|
 *		|
 *		|
 *	v	|
 *  v	z+ ---------------------> x
 *		<<		   column       >>
 */
class CubeCell
{
  public:
	CubeCell() : data_id_(0),
				 occu_(OccupancyType::OCCUPIED),
				 geo_mark_id_(0)
	{
	}

	CubeCell(uint64_t id, OccupancyType occupancy) : data_id_(id),
													 occu_(occupancy),
													 geo_mark_id_(0)
	{
	}

	uint64_t data_id_;
	Position3Di index_;
	Position3Dd location_;
	OccupancyType occu_;
	uint64_t geo_mark_id_;

	double GetHeuristic(const CubeCell &other_struct) const
	{
		double x1, y1, z1;
		double x2, y2, z2;

		x1 = this->index_.x;
		y1 = this->index_.y;
		z1 = this->index_.z;

		x2 = other_struct.index_.x;
		y2 = other_struct.index_.y;
		z2 = other_struct.index_.z;

		// static_cast: can get wrong result to use "unsigned long" type for deduction
		long x_error = static_cast<long>(x1) - static_cast<long>(x2);
		long y_error = static_cast<long>(y1) - static_cast<long>(y2);
		long z_error = static_cast<long>(z1) - static_cast<long>(z2);

		double cost = std::abs(x_error) + std::abs(y_error) + std::abs(z_error);
		//	std::cout<< "heuristic cost: " << cost << std::endl;

		return cost;
	}
};

class CubeArray
{
  public:
	// row, col, hei : x, y , z
	CubeArray(int32_t col_num, int32_t row_num, int32_t height_num, double cube_size);
	~CubeArray(){};

  public:
	int32_t row_size_;
	int32_t col_size_;
	int32_t hei_size_;

	// cube numbers on the negative side of axes
	int32_t row_offset_;
	int32_t col_offset_;
	int32_t hei_offset_;

	double cube_size_;

  public:
	bool isIDValid(uint64_t id)
	{
		if (id >= cubes_.size())
			return false;
		else
			return true;
	};

  public:
	std::vector<CubeCell> cubes_;

  public:
	void SetOriginOffset(int32_t col_offset, int32_t row_offset, int32_t hei_offset);
	uint64_t GetIDFromIndex(uint32_t col, uint32_t row, uint32_t hei);
	uint64_t GetIDFromPosition(double x, double y, double z);
	void SetCubeOccupancy(uint32_t col, uint32_t row, uint32_t hei, OccupancyType oc_type);
	void UpdateCubeOccupancy(double x, double y, double z, OccupancyType oc_type);
	std::vector<uint64_t> GetNeighbours(uint64_t id);
	std::vector<uint64_t> GetNeighbours(uint64_t id, bool allow_diag);
	std::vector<uint64_t> GetStartingCubes();
	bool GetCubeIDAtPosition(double x, double y, double z, uint64_t &id);

	bool GetCubeHeightIndexAtHeight(double z, std::vector<uint32_t> &hei_set);
	std::vector<uint64_t> GetFreeCubesAroundHeight(double height);
};
}

#endif /* CUBE_ARRAY_HPP */
