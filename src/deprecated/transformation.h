/*
 * librav_math.h
 *
 *  Created on: Sep 10, 2016
 *      Author: rdu
 */

#ifndef UTILS_TRANSFORMATION_H_
#define UTILS_TRANSFORMATION_H_

#include "eigen3/Eigen/Geometry"

#include "common/librav_types.h"

namespace librav {

namespace utils {

namespace Transformation {

typedef struct {
	Eigen::Translation<double,3> trans;
	Eigen::Quaterniond quat;
} Transform3D;

typedef Eigen::Translation<double,3> Translation3D;

Position3Dd TransformPosition3D(Transform3D transform, Position3Dd pos);

}

}

}

#endif /* UTILS_TRANSFORMATION_H_ */
