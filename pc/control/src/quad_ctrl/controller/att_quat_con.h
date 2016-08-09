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
#include "common/controller_base.h"
#include "quad_ctrl/controller/quadcon_io.h"

namespace srcl_ctrl {

class AttQuatCon: public Controller
{
public:
	AttQuatCon(RobotState *_rs);
	~AttQuatCon();

private:
	float kp_phi;
	float kd_phi;
	float kp_theta;
	float kd_theta;
	float kp_psi;
	float kd_psi;

	Eigen::Matrix<double,4,4> plus_type_trans_;
	Eigen::Matrix<double,4,4> plus_type_trans_inv_;
	Eigen::Matrix<double,4,4> x_type_trans_;
	Eigen::Matrix<double,4,4> x_type_trans_inv_;

private:
	Eigen::Matrix<double,4,1> CalcMotorCmd(Eigen::Matrix<float,4,1> force_toqure, QuadFlightType type);

public:
	void Update(ControlInput *input, ControlOutput *cmd);
};

}

#endif /* CONTROL_ATT_QUAT_CON_H_ */
