/*
 * cube_array_builder.cpp
 *
 *  Created on: Sep 8, 2016
 *      Author: rdu
 */

#include <cmath>
//#include <iomanip>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include "geometry/cube_array_builder.h"

using namespace srcl_ctrl;
using namespace octomap;

std::shared_ptr<CubeArray> CubeArrayBuilder::BuildCubeArrayFromOctree(std::shared_ptr<octomap::OcTree> tree)
{
	double mmin[3],mmax[3];
	double res = tree->getResolution();

	tree->getMetricMin(mmin[0],mmin[1],mmin[2]);
	tree->getMetricMax(mmax[0],mmax[1],mmax[2]);

	std::cout << "tree bound - min: \n" << mmin[0] << " , " << mmin[1] << " , " << mmin[2] << std::endl;
	std::cout << "tree bound - max: \n" << mmax[0] << " , " << mmax[1] << " , " << mmax[2] << std::endl;
	std::cout << "tree resolution: " << res << std::endl;
	std::cout << "measurement miss prob: " << tree->getProbMiss() << std::endl;

	int32_t row_size = (std::abs(mmin[0]) + std::abs(mmax[0]))/res;
	int32_t col_size = (std::abs(mmin[1]) + std::abs(mmax[1]))/res;
	int32_t hei_size = (std::abs(mmin[2]) + std::abs(mmax[2]))/res;

	int32_t row_offset = std::round(-mmin[0]/res);
	int32_t col_offset = std::round(-mmin[1]/res);
	int32_t hei_offset = std::round(-mmin[2]/res);
	//std::cout << std::setprecision(20) << -mmin[1]/res << '\n';
	//std::cout << "calc: " << -1.5/0.3 << " , col: " << - mmin[1]/res << " val: " << col_offset << std::endl;

	std::cout << "\nsize: " << row_size << " , " << col_size << " , " << hei_size << std::endl;
	std::cout << "offset: " << row_offset << " , " << col_offset << " , " << hei_offset << std::endl << std::endl;

	std::shared_ptr<CubeArray> cube_array = std::make_shared<CubeArray>(row_size,col_size,hei_size,tree->getResolution());
	cube_array->SetOriginOffset(row_offset, col_offset, hei_offset);

	//std::cout << "cube array size: " << cube_array->cubes_.size() << std::endl;

//	point3d queryt (res/2, res/2, res/2);
//	OcTreeNode* resultt = tree->search (queryt);
//	if (resultt != NULL)
//		std::cout << "occupancy probability at " << queryt << ":\t " << resultt->getOccupancy() << std::endl;

	for(auto& cube : cube_array->cubes_)
	{
//		if(cube.location_.x < mmin[0] || cube.location_.x > mmax[0] ||
//				cube.location_.y < mmin[1] || cube.location_.y > mmax[1] ||
//				cube.location_.z < mmin[2] || cube.location_.z > mmax[2])
//			continue;

		point3d query (cube.location_.x, cube.location_.y, cube.location_.z);

//		std::cout << "cube " << cube.data_id_ << " at ( "<< cube.index_.x << " , " << cube.index_.y << " , " << cube.index_.z << " ) : "
//				<< cube.location_.x << " , " << cube.location_.y << " , " << cube.location_.z << std::endl;

		OcTreeNode* result = tree->search (query);

		if (result != NULL) {
			//std::cout << "occupancy probability at " << query << ":\t " << result->getOccupancy() << std::endl;
			if(result->getOccupancy() < tree->getProbMiss())
				cube.occu_ = OccupancyType::FREE;
//			else if(result->getOccupancy() > tree->getProbHit())
//				cube.occu_ = OccupancyType::OCCUPIED;
		}
	}

	return cube_array;
}
