/*
 * cube_array_builder.h
 *
 *  Created on: Sep 8, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_GEOMETRY_CUBE_ARRAY_BUILDER_H_
#define PLANNING_SRC_GEOMETRY_CUBE_ARRAY_BUILDER_H_

#include <memory>
#include <cstdint>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include "map/cube_array.hpp"

namespace autodrive {

namespace CubeArrayBuilder {

std::shared_ptr<CubeArray> BuildEmptyCubeArray(int32_t row_size, int32_t col_size, int32_t hei_size, double res);
std::shared_ptr<CubeArray> BuildSolidCubeArray(int32_t row_size, int32_t col_size, int32_t hei_size, double res);
std::shared_ptr<CubeArray> BuildCubeArrayFromOctree(std::shared_ptr<octomap::OcTree> tree);
std::shared_ptr<CubeArray> BuildCubeArrayFromOctreeWithExtObstacle(std::shared_ptr<octomap::OcTree> tree);

}

}

#endif /* PLANNING_SRC_GEOMETRY_CUBE_ARRAY_BUILDER_H_ */
