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
#include "interface/controller.h"
#include "quad_ctrl/data_types/quad_state.h"

namespace librav {

struct AttQuatConParam
{
	float kp_phi;
	float kd_phi;
	float kp_theta;
	float kd_theta;
	float kp_psi;
	float kd_psi;
};

struct AttQuatConInput
{
	// quaternion
	Eigen::Quaterniond quat_d;
	// rotation rate
	float rot_rate_d[3];
	// total force
	// used to calculate motor commands
	float ftotal_d;
};

struct AttQuatConOutput
{
	float motor_ang_vel_d[4];
};

using AttQuatConIF = ControllerInterface<AttQuatConParam, QuadState,AttQuatConInput,AttQuatConOutput>;

class AttQuatCon: public AttQuatConIF
{
public:
	AttQuatCon(const QuadState& _rs);
	~AttQuatCon() = default;

private:
	// internal variables
	Eigen::Matrix<double,4,4> plus_type_trans_;
	Eigen::Matrix<double,4,4> plus_type_trans_inv_;
	Eigen::Matrix<double,4,4> x_type_trans_;
	Eigen::Matrix<double,4,4> x_type_trans_inv_;

	Eigen::Matrix<double,4,1> CalcMotorCmd(Eigen::Matrix<float,4,1> force_toqure, QuadFlightType type);

public:
	void InitParams(const AttQuatConParam& param) override;
	void Update(const AttQuatConInput& desired, AttQuatConOutput& cmd) override;
};

}

#endif /* CONTROL_ATT_QUAT_CON_H_ */
