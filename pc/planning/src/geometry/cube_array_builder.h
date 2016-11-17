/*
 * cube_array_builder.h
 *
 *  Created on: Sep 8, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_LOCAL3D_CUBE_ARRAY_BUILDER_H_
#define PLANNING_SRC_LOCAL3D_CUBE_ARRAY_BUILDER_H_

#include <memory>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include "cube_array/cube_array.h"

namespace srcl_ctrl {

namespace CubeArrayBuilder {

std::shared_ptr<CubeArray> BuildCubeArrayFromOctree(std::shared_ptr<octomap::OcTree> tree);
std::shared_ptr<CubeArray> BuildCubeArrayFromOctreeWithExtObstacle(std::shared_ptr<octomap::OcTree> tree);

}

}

#endif /* PLANNING_SRC_LOCAL3D_CUBE_ARRAY_BUILDER_H_ */
