/*
 * cube_array_builder.cpp
 *
 *  Created on: Sep 8, 2016
 *      Author: rdu
 */

#include <cmath>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include "local3d/cube_array_builder.h"

using namespace srcl_ctrl;
using namespace octomap;

std::shared_ptr<CubeArray> CubeArrayBuilder::BuildCubeArrayFromOctree(std::shared_ptr<octomap::OcTree> tree)
{
	double mmin[3],mmax[3];

	tree->getMetricMin(mmin[0],mmin[1],mmin[2]);
	tree->getMetricMax(mmax[0],mmax[1],mmax[2]);

	std::cout << "tree bound - min: \n" << mmin[0] << " , " << mmin[1] << " , " << mmin[2] << std::endl;
	std::cout << "tree bound - max: \n" << mmax[0] << " , " << mmax[1] << " , " << mmax[2] << std::endl;

	uint32_t col_size = (std::abs(mmin[0]) + std::abs(mmax[0]))/tree->getResolution();
	uint32_t row_size = (std::abs(mmin[1]) + std::abs(mmax[1]))/tree->getResolution();
	uint32_t hei_size = (std::abs(mmin[2]) + std::abs(mmax[2]))/tree->getResolution();

	int32_t col_offset = - mmin[0]/tree->getResolution();
	int32_t row_offset = - mmin[1]/tree->getResolution();
	int32_t hei_offset = - mmin[2]/tree->getResolution();
	std::cout << "size: " << col_size << " , " << row_size << " , " << hei_size << std::endl;
	std::cout << "offset: " << col_offset << " , " << row_offset << " , " << hei_offset << std::endl;

	std::shared_ptr<CubeArray> cube_array = std::make_shared<CubeArray>(col_size,row_size,hei_size,tree->getResolution());
	cube_array->SetOriginOffset(col_offset, row_offset, hei_offset);

	for(auto& cube : cube_array->cubes_)
	{
		if(cube.location_.x < mmin[0] || cube.location_.x > mmax[0] ||
				cube.location_.y < mmin[1] || cube.location_.y > mmax[1] ||
				cube.location_.z < mmin[2] || cube.location_.y > mmax[2])
			continue;

		point3d query (cube.location_.x, cube.location_.y, cube.location_.z);

		OcTreeNode* result = tree->search (query);

		if (result != NULL) {
			//std::cout << "occupancy probability at " << query << ":\t " << result->getOccupancy() << std::endl;
			if(result->getOccupancy() < 0.4)
				cube.occu_ = OccupancyType::FREE;
		}
	}

	return cube_array;
}
