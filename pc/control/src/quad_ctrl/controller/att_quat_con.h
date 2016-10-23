/*
 * att_quat_con.h
 *
 *  Created on: Mar 3, 2016
 *      Author: rdu
 */

#ifndef CONTROL_ATT_QUAT_CON_H_
#define CONTROL_ATT_QUAT_CON_H_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "common/control_types.h"
#include "quad_ctrl/data_types/quad_state.h"

namespace srcl_ctrl {

typedef struct
{
	// input of position controller
	float pos_d[3];
	float vel_d[3];
	float acc_d[3];
	float yaw_d;
	float yaw_rate_d;

	// input of attitude controller (using Euler)
	float euler_d[3];
	float rot_rate_d[3];
	float delta_w_F;

	// input of attitude controller (using Quaternion)
	Eigen::Quaterniond quat_d;
	float ftotal_d;

}AttQuatConInput;

typedef struct{
	// output of position controller (using Euler)
	float euler_d[3];
	float delta_w_F;

	// output of position controller (using Quaternion)
	Eigen::Quaterniond quat_d;
	float ftotal_d;

	// output of attitude controller
	float motor_ang_vel_d[3];
}AttQuatConOutput;

class AttQuatCon
{
public:
	AttQuatCon(const QuadState& _rs);
	~AttQuatCon();

private:
	float kp_phi;
	float kd_phi;
	float kp_theta;
	float kd_theta;
	float kp_psi;
	float kd_psi;

private:
	const QuadState& rs_;

	// internal variables
	Eigen::Matrix<double,4,4> plus_type_trans_;
	Eigen::Matrix<double,4,4> plus_type_trans_inv_;
	Eigen::Matrix<double,4,4> x_type_trans_;
	Eigen::Matrix<double,4,4> x_type_trans_inv_;

	Eigen::Matrix<double,4,1> CalcMotorCmd(Eigen::Matrix<float,4,1> force_toqure, QuadFlightType type);

public:
	void SetControlGains(float _kp_phi, float _kd_phi, float _kp_theta, float _kd_theta, float _kp_psi, float _kd_psi)
	{
		kp_phi = _kp_phi;
		kd_phi = _kd_phi;
		kp_theta = _kp_theta;
		kd_theta = _kd_theta;
		kp_psi = _kp_psi;
		kd_psi = _kd_psi;
	};

	void Update(const AttQuatConInput& input, AttQuatConOutput& output);
};

}

#endif /* CONTROL_ATT_QUAT_CON_H_ */
