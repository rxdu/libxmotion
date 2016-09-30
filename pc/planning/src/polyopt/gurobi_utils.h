/*
 * gurobi_utils.h
 *
 *  Created on: Aug 28, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_POLYOPT_GUROBI_UTILS_H_
#define PLANNING_SRC_POLYOPT_GUROBI_UTILS_H_

#include <cstdint>
#include <vector>

#include "gurobi_c++.h"
#include "eigen3/Eigen/Core"

namespace srcl_ctrl {

namespace GurobiUtils {

void GetQuadraticCostFuncExpr(const std::vector<GRBVar>& x, const Eigen::Ref<const Eigen::MatrixXf> Q, uint32_t x_size, GRBQuadExpr& expr);
void AddLinEqualityConstrExpr(const std::vector<GRBVar>& x, const Eigen::Ref<const Eigen::MatrixXf> A_eq, const Eigen::Ref<const Eigen::MatrixXf> b_eq,
				uint32_t x_size, uint32_t constr_size, GRBModel& model);
void AddLinInequalityConstrExpr(const std::vector<GRBVar>& x, const Eigen::Ref<const Eigen::MatrixXf> A_ineq, const Eigen::Ref<const Eigen::MatrixXf> b_ineq,
				uint32_t x_size, uint32_t constr_size, GRBModel& model);
}

}

#endif /* PLANNING_SRC_POLYOPT_GUROBI_UTILS_H_ */
