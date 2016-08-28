/*
 * polyopt_utils.h
 *
 *  Created on: Aug 23, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_POLYOPT_POLYOPT_MATH_H_
#define PLANNING_SRC_POLYOPT_POLYOPT_MATH_H_

#include <cstdint>

#include "eigen3/Eigen/Core"

namespace srcl_ctrl {

using PolynomialCoeffs = Eigen::Array<float,1, Eigen::Dynamic>;

namespace PolyOptMath {

void GetDimQMatrix(uint32_t poly_order, uint32_t deriv_order, double t0, double t1, Eigen::Ref<Eigen::MatrixXf> q);
void GetNonDimQMatrix(uint32_t poly_order, uint32_t deriv_order, double t0, double t1, Eigen::Ref<Eigen::MatrixXf> q);
void GetNonDimQMatrices(uint32_t poly_order, uint32_t deriv_order, uint32_t keyframe_num,
		const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts, Eigen::Ref<Eigen::MatrixXf> q);

void GetDerivativeCoeffs(uint32_t poly_order, uint32_t deriv_order, Eigen::Ref<PolynomialCoeffs> coeffs);
uint32_t GetNonZeroCoeffNum(const Eigen::Ref<const PolynomialCoeffs> coeffs);
void GetNonDimEqualityConstrs(uint32_t poly_order, uint32_t deriv_order, uint32_t keyframe_num,
		const Eigen::Ref<const Eigen::MatrixXf> keyframe_vals, const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts,
		Eigen::Ref<Eigen::MatrixXf> A_eq, Eigen::Ref<Eigen::MatrixXf> b_eq);
}
}

#endif /* PLANNING_SRC_POLYOPT_POLYOPT_MATH_H_ */
