/*
 * cube_array.h
 *
 *  Created on: Sep 8, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_CUBE_ARRAY_CUBE_ARRAY_H_
#define PLANNING_SRC_CUBE_ARRAY_CUBE_ARRAY_H_

#include <map>
#include <vector>

#include "common/planning_types.h"
#include "graph/bds_base.h"

namespace srcl_ctrl{

class CubeCell: public BDSBase<CubeCell>{
public:
	CubeCell():
		BDSBase<CubeCell>(0),
		occu_(OccupancyType::OCCUPIED),
		geo_mark_id_(0)
		{

		}

	CubeCell(uint64_t id, OccupancyType occupancy):
		BDSBase<CubeCell>(id),
		occu_(occupancy),
		geo_mark_id_(0)
		{

		}

	Position3D index_;
	Position3Dd location_;
	OccupancyType occu_;
	uint64_t geo_mark_id_;

	double GetHeuristic(const CubeCell& other_struct) const{
		double x1,y1,z1;
		double x2,y2,z2;

		x1 = this->location_.x;
		y1 = this->location_.y;
		z1 = this->location_.z;

		x2 = other_struct.location_.x;
		y2 = other_struct.location_.y;
		z2 = other_struct.location_.z;

		// static_cast: can get wrong result to use "unsigned long" type for deduction
		long x_error = static_cast<long>(x1) - static_cast<long>(x2);
		long y_error = static_cast<long>(y1) - static_cast<long>(y2);
		long z_error = static_cast<long>(z1) - static_cast<long>(z2);

		double cost = std::abs(x_error) + std::abs(y_error) + std::abs(z_error);
		//	std::cout<< "heuristic cost: " << cost << std::endl;

		return cost;
	}
};

class CubeArray {
public:
	// row, col, hei : x, y , z
	CubeArray(uint32_t row_num, uint32_t col_num, uint32_t height_num, double cube_size);
	~CubeArray(){};

public:
	uint32_t row_size_;
	uint32_t col_size_;
	uint32_t hei_size_;

	// cube numbers on the negative side of axes
	int32_t row_offset_;
	int32_t col_offset_;
	int32_t hei_offset_;

	double cube_size_;

public:
	std::vector<CubeCell> cubes_;

public:
	void SetOriginOffset(int32_t row_offset, int32_t col_offset, int32_t hei_offset);
	uint64_t GetIDFromIndex(uint32_t row, uint32_t col, uint32_t hei);
	uint64_t GetIDFromPosition(double x, double y, double z);
	void UpdateCubeOccupancy(double x, double y, double z, OccupancyType oc_type);
	std::vector<uint64_t> GetNeighbours(uint64_t id);
	std::vector<uint64_t> GetStartingCubes();
};

}

#endif /* PLANNING_SRC_CUBE_ARRAY_CUBE_ARRAY_H_ */
