/*
 * polyopt_utils.h
 *
 *  Created on: Aug 23, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_POLYOPT_POLYOPT_UTILS_H_
#define PLANNING_SRC_POLYOPT_POLYOPT_UTILS_H_

#include <cstdint>

#include "eigen3/Eigen/Core"

namespace srcl_ctrl {

class PolyOptUtils {
public:
	PolyOptUtils(){};
	~PolyOptUtils(){};

public:
	void GetDerivativeCoeffs(uint32_t poly_order, uint32_t deriv_order, Eigen::Ref<Eigen::ArrayXXf>& coeffs);
};

}

#endif /* PLANNING_SRC_POLYOPT_POLYOPT_UTILS_H_ */
