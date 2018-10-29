/* 
 * osqp_polyopt.hpp
 * 
 * Created on: Apr 03, 2018 13:24
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef OSQP_POLYOPT_HPP
#define OSQP_POLYOPT_HPP

#include <cstdint>
#include <iostream>
#include <string>

#include "gurobi_c++.h"
#include "eigen3/Eigen/Core"

#include "polyopt/polyopt_types.hpp"

namespace librav
{
class OSQPPolyOpt
{
  public:
	OSQPPolyOpt() = default;
	~OSQPPolyOpt() = default;

	// optimize a one-dimensional trajectory, returns a curve data structure
	OptResultCurve OptimizeTrajectory(const Eigen::Ref<const Eigen::MatrixXf> keyframe_vals,
									  const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts, uint32_t keyframe_num, uint32_t poly_order, uint32_t deriv_order);

	// jointly optimize a multi-dimensional trajectory with only equality constraints, returns optimized parameters
	OptResultParam OptimizeTrajectory(const Eigen::Ref<const Eigen::MatrixXf> Q_m,
									  const Eigen::Ref<const Eigen::MatrixXf> Aeq_m, const Eigen::Ref<const Eigen::MatrixXf> beq_m,
									  const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts,
									  uint32_t keyframe_num, uint32_t var_size);

	// jointly optimize a multi-dimensional trajectory with both equality and inequality constraints, returns optimized parameters
	OptResultParam OptimizeTrajectory(const Eigen::Ref<const Eigen::MatrixXf> Q_m,
									  const Eigen::Ref<const Eigen::MatrixXf> Aeq_m, const Eigen::Ref<const Eigen::MatrixXf> beq_m,
									  const Eigen::Ref<const Eigen::MatrixXf> Aineq_m, const Eigen::Ref<const Eigen::MatrixXf> bineq_m,
									  const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts,
									  uint32_t keyframe_num, uint32_t var_size);

  private:
	// derivative to optimize
	uint32_t r_ = 4;
	// highest order of polynomial
	uint32_t N_ = 10;
	// number of key frames
	uint32_t kf_num_ = 2;

	Eigen::MatrixXf Q_;
	Eigen::MatrixXf A_eq_;
	Eigen::MatrixXf b_eq_;

	void InitCalcVars();
};
}
#endif /* OSQP_POLYOPT_HPP */
