/* 
 * osqp_polyopt.cpp
 * 
 * Created on: Apr 03, 2018 13:27
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "polyopt/osqp_solver/osqp_polyopt.hpp"

using namespace librav;

OptResultCurve OSQPPolyOpt::OptimizeTrajectory(const Eigen::Ref<const Eigen::MatrixXf> keyframe_vals,
                                               const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts, uint32_t keyframe_num, uint32_t poly_order, uint32_t deriv_order)
{
}