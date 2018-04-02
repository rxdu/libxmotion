/* 
 * gurobi_polyopt.hpp
 * 
 * Created on: Aug 29, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef GUROBI_POLYOPT_HPP
#define GUROBI_POLYOPT_HPP

#include <cstdint>
#include <iostream>
#include <string>

#include "gurobi_c++.h"
#include "eigen3/Eigen/Core"

#include "common/poly_curve.hpp"

namespace librav
{
struct OptResultCurve
{
	std::vector<CurveParameter> segments;
	double cost;

	void print()
	{
		std::cout << "\n**************************************\n"
				  << std::endl;
		std::cout << "cost: " << cost << std::endl;

		uint32_t idx = 0;
		for (auto &seg : segments)
		{
			std::cout << "\nseg " << idx << " : " << std::endl;
			uint32_t coeff_idx = 0;
			for (auto &coef : seg.coeffs)
				std::cout << coeff_idx++ << " : " << coef << std::endl;
			std::cout << "start time: " << seg.ts << " , end time: " << seg.te << std::endl;
			idx++;
		}
		std::cout << "\n**************************************\n"
				  << std::endl;
	}
};

struct OptResultParam
{
	std::vector<double> params;
	double cost;
};

class GurobiPolyOpt
{
  public:
	GurobiPolyOpt() = default;
	~GurobiPolyOpt() = default;

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
	// only one environment needed
	static GRBEnv grb_env_;

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

#endif /* GUROBI_POLYOPT_HPP */
