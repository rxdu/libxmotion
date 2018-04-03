/* 
 * quad_polyopt.hpp
 * 
 * Created on: Aug 29, 2016
 * Description: this configuration tries to find the minimum snap trajectory
 *  	for a quadrotor in the flat output space (x,y,z,yaw).
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */
#ifndef QUAD_POLYOPT_HPP
#define QUAD_POLYOPT_HPP

#include <cstdint>

#include <gurobi_c++.h>
#include "eigen3/Eigen/Core"

#include "common/quad_flattraj.hpp"
#include "polyopt/gurobi_solver/gurobi_polyopt.hpp"

namespace librav
{

typedef struct
{
	uint64_t Qpos_size;
	uint64_t Qyaw_size;

	uint64_t Aeq_pos_size_r;
	uint64_t Aeq_pos_size_c;
	uint64_t Aeq_yaw_size_r;
	uint64_t Aeq_yaw_size_c;

	uint64_t beq_pos_size;
	uint64_t beq_yaw_size;

	uint64_t var_size;

	// corridor
	uint64_t midpoint_num;
	double corridor_size;
	uint64_t var_cor_pos_size;
	uint64_t var_cor_yaw_size;
	uint64_t cor_constr_size;
} OptMatrixSize;

class QuadPolyOpt
{
  public:
	QuadPolyOpt() = default;
	~QuadPolyOpt() = default;

	// input of trajectory optimization
	Eigen::MatrixXf keyframe_x_vals_;
	Eigen::MatrixXf keyframe_y_vals_;
	Eigen::MatrixXf keyframe_z_vals_;
	Eigen::MatrixXf keyframe_yaw_vals_;
	Eigen::MatrixXf keyframe_ts_;

	// output of trajectory optimization
	QuadFlatTraj flat_traj_;

	// optimize each dimension of the trajectory separately (not recommended)
	void InitOptMatrices(uint32_t keyframe_num);
	void OptimizeFlatTraj();
	void OptimizeFlatTraj(const Eigen::Ref<const Eigen::MatrixXf> keyframe_x_vals,
						  const Eigen::Ref<const Eigen::MatrixXf> keyframe_y_vals,
						  const Eigen::Ref<const Eigen::MatrixXf> keyframe_z_vals,
						  const Eigen::Ref<const Eigen::MatrixXf> keyframe_yaw_vals,
						  const Eigen::Ref<const Eigen::MatrixXf> keyframe_ts,
						  uint32_t keyframe_num);

	// jointly optimize all dimensions of the trajectory WITHOUT corridor constraint
	void InitOptJointMatrices(uint32_t keyframe_num);
	void OptimizeFlatTrajJoint();

	// jointly optimize all dimensions of the trajectory WITH corridor constraint
	void InitOptWithCorridorJointMatrices(uint32_t keyframe_num, uint32_t midpoint_num, double cor_size);
	bool OptimizeFlatTrajWithCorridorJoint();

  private:
	GurobiPolyOpt optimizer_;

	const uint32_t r_pos_ = 4;
	uint32_t N_pos_ = 10;

	const uint32_t r_yaw_ = 2;
	uint32_t N_yaw_ = 3;

	uint32_t keyframe_num_ = 2;

	OptResultCurve traj_[4];

	// for joint optimization
	OptMatrixSize opt_size_;

	Eigen::MatrixXf Q_x_;
	Eigen::MatrixXf Q_y_;
	Eigen::MatrixXf Q_z_;
	Eigen::MatrixXf Q_yaw_;

	Eigen::MatrixXf Aeq_x_;
	Eigen::MatrixXf Aeq_y_;
	Eigen::MatrixXf Aeq_z_;
	Eigen::MatrixXf Aeq_yaw_;

	Eigen::MatrixXf beq_x_;
	Eigen::MatrixXf beq_y_;
	Eigen::MatrixXf beq_z_;
	Eigen::MatrixXf beq_yaw_;

	Eigen::MatrixXf Q_joint_;
	Eigen::MatrixXf Aeq_joint_;
	Eigen::MatrixXf beq_joint_;

	// corridor constraints
	std::vector<Eigen::MatrixXf> corridor_frames_;

	Eigen::MatrixXf A_cor_;
	Eigen::MatrixXf b_cor_;

	void SetPositionPolynomialOrder(uint32_t N) { N_pos_ = N; };
	uint32_t GetPositionPolynomialOrder() { return N_pos_; };
	void SetYawPolynomialOrder(uint32_t N) { N_yaw_ = N; };
	uint32_t GetYawPolynomialOrder() { return N_yaw_; };
};
}

#endif /* QUAD_POLYOPT_HPP */
